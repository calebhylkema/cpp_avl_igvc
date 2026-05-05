// =============================================================================
// igvc_control  —  IgvcHardware  —  AVL IGVC 2026
// =============================================================================
//
// ros2_control SystemInterface for the IGVC drivetrain.
// Owns the Teensy USB serial port.  Sends velocity commands, reads encoder
// RPM and position feedback.
//
// Drivetrain: NEO Vortex → 12.75:1 gearbox → 1:1 chain → sprocket (r=0.0345m)
//
// Firmware serial protocol:
//   TX: "L<rpm> R<rpm>\n"                       motor RPM setpoints
//   TX: "S\n"                                   stop
//   RX: "E L<rpm> <pos> R<rpm> <pos>\n"         encoder feedback
//
// Conversion:
//   motor_rpm    = wheel_rad_s * gear_ratio * 60 / (2π)
//   wheel_rad_s  = motor_rpm / gear_ratio * 2π / 60
//   wheel_rad    = motor_rotations / gear_ratio * 2π
// =============================================================================

#include "igvc_control/igvc_hardware.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace igvc_control
{

// ── Helpers ──────────────────────────────────────────────────────────────────

static constexpr double TWO_PI         = 2.0 * M_PI;
static constexpr double TWO_PI_OVER_60 = TWO_PI / 60.0;

// ── Lifecycle ────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn IgvcHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    serial_port_    = info_.hardware_parameters.count("serial_port")
                        ? info_.hardware_parameters.at("serial_port")
                        : "/dev/ttyACM0";
    gear_ratio_     = info_.hardware_parameters.count("gear_ratio")
                        ? std::stod(info_.hardware_parameters.at("gear_ratio"))
                        : 12.75;
    max_motor_rpm_  = info_.hardware_parameters.count("max_motor_rpm")
                        ? std::stod(info_.hardware_parameters.at("max_motor_rpm"))
                        : 3000.0;
    left_inverted_  = info_.hardware_parameters.count("left_motor_inverted")
                        ? (info_.hardware_parameters.at("left_motor_inverted") == "true")
                        : false;
    right_inverted_ = info_.hardware_parameters.count("right_motor_inverted")
                        ? (info_.hardware_parameters.at("right_motor_inverted") == "true")
                        : true;

    RCLCPP_INFO(logger_, "serial_port=%s  gear_ratio=%.2f  max_motor_rpm=%.0f  left_inv=%d  right_inv=%d",
                serial_port_.c_str(), gear_ratio_, max_motor_rpm_, left_inverted_, right_inverted_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IgvcHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    openSerial();

    // Set SparkMAX PID gains (same as actuator_node)
    const char *pid_cmds[] = {"KF0.000197\n", "KP0.0004\n", "KI0.0\n", "KD0.0\n"};
    for (auto cmd : pid_cmds) {
        writeSerial(cmd);
        usleep(200000);
    }
    RCLCPP_INFO(logger_, "SparkMAX PID gains set");

    for (int i = 0; i < 2; ++i) {
        hw_positions_[i]  = 0.0;
        hw_velocities_[i] = 0.0;
        hw_commands_[i]   = 0.0;
        fb_positions_[i]  = 0.0;
        fb_velocities_[i] = 0.0;
    }

    running_ = true;
    reader_thread_ = std::thread(&IgvcHardware::readerThread, this);

    RCLCPP_INFO(logger_, "Activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IgvcHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    running_ = false;
    if (reader_thread_.joinable()) reader_thread_.join();

    writeSerial("S\n");
    closeSerial();
    RCLCPP_INFO(logger_, "Deactivated — motors stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Interfaces ───────────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface> IgvcHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.emplace_back("left_wheel_joint",  hardware_interface::HW_IF_POSITION, &hw_positions_[0]);
    interfaces.emplace_back("left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]);
    interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[1]);
    interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]);
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> IgvcHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.emplace_back("left_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
    interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);
    return interfaces;
}

// ── Read: copy latest feedback from reader thread ───────────────────────────

hardware_interface::return_type IgvcHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(fb_mtx_);
    hw_velocities_[0] = fb_velocities_[0];
    hw_velocities_[1] = fb_velocities_[1];
    hw_positions_[0]  = fb_positions_[0];
    hw_positions_[1]  = fb_positions_[1];
    return hardware_interface::return_type::OK;
}

// ── Write: send velocity command to Teensy ───────────────────────────────────

hardware_interface::return_type IgvcHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    double rpm_left  = (hw_commands_[0] / TWO_PI_OVER_60) * gear_ratio_;
    double rpm_right = (hw_commands_[1] / TWO_PI_OVER_60) * gear_ratio_;

    if (left_inverted_)  rpm_left  = -rpm_left;
    if (right_inverted_) rpm_right = -rpm_right;

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "L%.0f R%.0f\n", rpm_left, rpm_right);
    writeSerial(cmd);

    return hardware_interface::return_type::OK;
}

// ── Background reader thread ────────────────────────────────────────────────

void IgvcHardware::readerThread()
{
    char rx_buf[256];
    int rx_len = 0;
    char tmp[256];

    while (running_) {
        ssize_t n;
        {
            std::lock_guard<std::mutex> lock(serial_mtx_);
            if (serial_fd_ < 0) { usleep(10000); continue; }
            n = ::read(serial_fd_, tmp, sizeof(tmp));
        }

        if (n <= 0) {
            usleep(1000);
            continue;
        }

        for (int i = 0; i < n; ++i) {
            char c = tmp[i];
            if (c == '\n') {
                rx_buf[rx_len] = '\0';
                if (rx_len > 0 && (rx_buf[0] == 'E' || rx_buf[0] == 'e')) {
                    float rpm_l = 0.0f, pos_l = 0.0f, rpm_r = 0.0f, pos_r = 0.0f;
                    char *lp = strchr(rx_buf, 'L'); if (!lp) lp = strchr(rx_buf, 'l');
                    char *rp = strchr(rx_buf, 'R'); if (!rp) rp = strchr(rx_buf, 'r');
                    if (lp && sscanf(lp + 1, "%f %f", &rpm_l, &pos_l) >= 1 &&
                        rp && sscanf(rp + 1, "%f %f", &rpm_r, &pos_r) >= 1) {
                        if (left_inverted_)  { rpm_l = -rpm_l; pos_l = -pos_l; }
                        if (right_inverted_) { rpm_r = -rpm_r; pos_r = -pos_r; }
                        std::lock_guard<std::mutex> lock(fb_mtx_);
                        fb_velocities_[0] = (rpm_l / gear_ratio_) * TWO_PI_OVER_60;
                        fb_velocities_[1] = (rpm_r / gear_ratio_) * TWO_PI_OVER_60;
                        fb_positions_[0] = (pos_l / gear_ratio_) * TWO_PI;
                        fb_positions_[1] = (pos_r / gear_ratio_) * TWO_PI;
                    }
                }
                rx_len = 0;
            } else if (rx_len < (int)(sizeof(rx_buf) - 1)) {
                rx_buf[rx_len++] = c;
            }
        }
    }
}

// ── Serial helpers ───────────────────────────────────────────────────────────

void IgvcHardware::openSerial()
{
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_WARN(logger_, "Could not open %s — running without hardware",
                    serial_port_.c_str());
        return;
    }

    struct termios tio{};
    tcgetattr(serial_fd_, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(serial_fd_, TCSANOW, &tio);
    tcflush(serial_fd_, TCIOFLUSH);

    RCLCPP_INFO(logger_, "Serial port %s opened", serial_port_.c_str());
}

void IgvcHardware::closeSerial()
{
    std::lock_guard<std::mutex> lock(serial_mtx_);
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

void IgvcHardware::writeSerial(const std::string & data)
{
    std::lock_guard<std::mutex> lock(serial_mtx_);
    if (serial_fd_ >= 0) {
        ::write(serial_fd_, data.c_str(), data.size());
    }
}

}  // namespace igvc_control

PLUGINLIB_EXPORT_CLASS(igvc_control::IgvcHardware, hardware_interface::SystemInterface)
