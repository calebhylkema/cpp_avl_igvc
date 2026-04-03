// =============================================================================
// igvc_hw  —  serial_bridge  —  AVL IGVC 2026
// =============================================================================
//
// Single owner of the Teensy 4.1 USB serial port.
// All other nodes (teleop, nav cmd_vel converter) publish motor command strings
// to /igvc/motor_cmd — this node writes them straight to the Teensy.
//
// Subscribed topics:
//   /igvc/motor_cmd   std_msgs/String   motor command string from any source
//
// Serial protocol (matches igvc_motor_firmware):
//   "M <left_duty> <right_duty>\n"   duty cycles ±1.0
//   "S\n"                            stop
//
// Watchdog: if no /igvc/motor_cmd is received within cmd_timeout seconds,
// sends "S\n" to the Teensy so the robot stops safely when a command source dies.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// POSIX serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

class SerialBridgeNode : public rclcpp::Node
{
public:
    SerialBridgeNode()
    : rclcpp::Node("igvc_serial_bridge")
    {
        declare_parameter<std::string>("serial_port",      "/dev/ttyACM0");
        declare_parameter<int>        ("serial_baud_rate", 115200);
        declare_parameter<double>     ("cmd_timeout",      0.5);

        serial_port_      = get_parameter("serial_port").as_string();
        serial_baud_rate_ = get_parameter("serial_baud_rate").as_int();
        cmd_timeout_      = get_parameter("cmd_timeout").as_double();

        sub_ = create_subscription<std_msgs::msg::String>(
            "/igvc/motor_cmd", 10,
            std::bind(&SerialBridgeNode::motorCmdCb, this, std::placeholders::_1));

        // Watchdog timer: checks for command timeout at 10 Hz
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SerialBridgeNode::watchdogCb, this));

        openSerial();
        last_cmd_time_ = now();

        RCLCPP_INFO(get_logger(), "serial_bridge ready  port=%s  baud=%d  timeout=%.1fs",
            serial_port_.c_str(), serial_baud_rate_, cmd_timeout_);
    }

    ~SerialBridgeNode()
    {
        writeSerial("S\n");
        if (serial_fd_ >= 0) ::close(serial_fd_);
    }

private:
    // ── Callbacks ─────────────────────────────────────────────────────────
    void motorCmdCb(const std_msgs::msg::String::SharedPtr msg)
    {
        last_cmd_time_ = now();
        writeSerial(msg->data);
    }

    void watchdogCb()
    {
        const double elapsed = (now() - last_cmd_time_).seconds();
        if (elapsed > cmd_timeout_) {
            writeSerial("S\n");
        }
    }

    // ── Serial helpers ────────────────────────────────────────────────────
    void openSerial()
    {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            RCLCPP_WARN(get_logger(),
                "Could not open serial port %s — motor commands will be dropped",
                serial_port_.c_str());
            return;
        }

        struct termios tio{};
        tcgetattr(serial_fd_, &tio);
        cfmakeraw(&tio);

        speed_t baud = B115200;
        if      (serial_baud_rate_ == 9600)   baud = B9600;
        else if (serial_baud_rate_ == 57600)  baud = B57600;
        else if (serial_baud_rate_ == 115200) baud = B115200;
        else if (serial_baud_rate_ == 230400) baud = B230400;
        else if (serial_baud_rate_ == 921600) baud = B921600;
        else RCLCPP_WARN(get_logger(),
            "Unrecognised serial_baud_rate %d, defaulting to 115200", serial_baud_rate_);

        cfsetispeed(&tio, baud);
        cfsetospeed(&tio, baud);
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;
        tcsetattr(serial_fd_, TCSANOW, &tio);
        tcflush(serial_fd_, TCIOFLUSH);

        RCLCPP_INFO(get_logger(), "Serial port %s opened at %d baud",
            serial_port_.c_str(), serial_baud_rate_);
    }

    void writeSerial(const std::string & msg)
    {
        if (serial_fd_ >= 0) {
            ::write(serial_fd_, msg.c_str(), msg.size());
        }
    }

    // ── Members ───────────────────────────────────────────────────────────
    std::string serial_port_;
    int         serial_baud_rate_ = 115200;
    double      cmd_timeout_      = 0.5;

    int serial_fd_ = -1;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr                           watchdog_timer_;
    rclcpp::Time                                           last_cmd_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
