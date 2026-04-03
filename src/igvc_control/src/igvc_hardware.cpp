// =============================================================================
// igvc_control  —  IgvcHardware  —  AVL IGVC 2026
// =============================================================================
//
// ros2_control SystemInterface for the IGVC drivetrain.
// Owns the Teensy USB serial port.  Sends velocity commands, reads encoder RPM.
//
// Drivetrain: NEO Vortex → 12.75:1 gearbox → 1:1 chain → sprocket (r=0.0345m)
//
// Firmware serial protocol:
//   TX: "M <duty_left> <duty_right>\n"   duty ±1.0 (mapped from wheel rad/s)
//   TX: "S\n"                            stop
//   RX: "E <motor_rpm_left> <motor_rpm_right>\n"   encoder feedback
//
// Conversion:
//   wheel_rad_s  = motor_rpm / gear_ratio * 2π / 60
//   duty         = (wheel_rad_s / max_wheel_rad_s)  clamped to ±1.0
//   max_wheel_rad_s = max_motor_rpm / gear_ratio * 2π / 60
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

static constexpr double TWO_PI_OVER_60 = 2.0 * M_PI / 60.0;

// ── Lifecycle ────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn IgvcHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Read hardware parameters from URDF <ros2_control> tag
    serial_port_    = info_.hardware_parameters.count("serial_port")
                        ? info_.hardware_parameters.at("serial_port")
                        : "/dev/ttyACM0";
    gear_ratio_     = info_.hardware_parameters.count("gear_ratio")
                        ? std::stod(info_.hardware_parameters.at("gear_ratio"))
                        : 12.75;
    max_motor_rpm_  = info_.hardware_parameters.count("max_motor_rpm")
                        ? std::stod(info_.hardware_parameters.at("max_motor_rpm"))
                        : 5700.0;
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

    // Zero state
    for (int i = 0; i < 2; ++i) {
        hw_positions_[i]  = 0.0;
        hw_velocities_[i] = 0.0;
        hw_commands_[i]   = 0.0;
    }

    RCLCPP_INFO(logger_, "Activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IgvcHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
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

// ── Read: parse encoder feedback from Teensy ─────────────────────────────────

hardware_interface::return_type IgvcHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    // Drain serial RX buffer, parse complete lines
    char tmp[128];
    int n = readSerial(tmp, sizeof(tmp));
    for (int i = 0; i < n; ++i) {
        char c = tmp[i];
        if (c == '\n') {
            rx_buf_[rx_len_] = '\0';
            if (rx_len_ > 0 && (rx_buf_[0] == 'E' || rx_buf_[0] == 'e')) {
                float motor_rpm_left = 0.0f, motor_rpm_right = 0.0f;
                if (sscanf(rx_buf_ + 1, "%f %f", &motor_rpm_left, &motor_rpm_right) == 2) {
                    // Apply motor inversion to encoder feedback
                    if (left_inverted_)  motor_rpm_left  = -motor_rpm_left;
                    if (right_inverted_) motor_rpm_right = -motor_rpm_right;
                    // Convert motor RPM → wheel rad/s
                    hw_velocities_[0] = (motor_rpm_left  / gear_ratio_) * TWO_PI_OVER_60;
                    hw_velocities_[1] = (motor_rpm_right / gear_ratio_) * TWO_PI_OVER_60;
                }
            }
            rx_len_ = 0;
        } else if (rx_len_ < (int)(sizeof(rx_buf_) - 1)) {
            rx_buf_[rx_len_++] = c;
        }
    }

    // Integrate position from velocity
    double dt = period.seconds();
    hw_positions_[0] += hw_velocities_[0] * dt;
    hw_positions_[1] += hw_velocities_[1] * dt;

    return hardware_interface::return_type::OK;
}

// ── Write: send velocity command to Teensy ───────────────────────────────────

hardware_interface::return_type IgvcHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Convert wheel rad/s command → duty [-1, 1]
    // max_wheel_rad_s = max_motor_rpm / gear_ratio * (2π/60)
    double max_wheel_rad_s = (max_motor_rpm_ / gear_ratio_) * TWO_PI_OVER_60;

    double duty_left  = hw_commands_[0] / max_wheel_rad_s;
    double duty_right = hw_commands_[1] / max_wheel_rad_s;

    // Apply motor inversion
    if (left_inverted_)  duty_left  = -duty_left;
    if (right_inverted_) duty_right = -duty_right;

    // Clamp
    duty_left  = std::max(-1.0, std::min(1.0, duty_left));
    duty_right = std::max(-1.0, std::min(1.0, duty_right));

    // Send to Teensy
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "M %.4f %.4f\n", duty_left, duty_right);
    writeSerial(cmd);

    return hardware_interface::return_type::OK;
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
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
}

void IgvcHardware::writeSerial(const std::string & data)
{
    if (serial_fd_ >= 0) {
        ::write(serial_fd_, data.c_str(), data.size());
    }
}

int IgvcHardware::readSerial(char * buf, size_t len)
{
    if (serial_fd_ < 0) return 0;
    ssize_t n = ::read(serial_fd_, buf, len);
    return (n > 0) ? static_cast<int>(n) : 0;
}

}  // namespace igvc_control

PLUGINLIB_EXPORT_CLASS(igvc_control::IgvcHardware, hardware_interface::SystemInterface)
