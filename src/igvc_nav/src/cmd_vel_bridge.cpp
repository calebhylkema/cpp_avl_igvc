// =============================================================================
// igvc_nav  —  cmd_vel_bridge  —  AVL IGVC 2026
// =============================================================================
//
// Converts Nav2 MPPI /cmd_vel (geometry_msgs/Twist) to the IGVS motor command
// string format and publishes to /igvc/motor_cmd.
// igvc_hw/serial_bridge subscribes to that topic and writes to the Teensy.
//
// Subscribed topics:
//   /cmd_vel          geometry_msgs/Twist   velocity from Nav2 MPPI controller
//
// Published topics:
//   /igvc/motor_cmd   std_msgs/String       "M <left_duty> <right_duty>\n" or "S\n"
//
// Differential drive kinematics:
//   v_left  = linear.x  −  angular.z × (wheel_base / 2)
//   v_right = linear.x  +  angular.z × (wheel_base / 2)
//   duty    = v / max_linear_speed   (clamped ±1.0)
//
// Watchdog: if no /cmd_vel arrives within cmd_timeout seconds, publishes "S\n"
// so the serial_bridge stops the robot if Nav2 goes silent.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdio>
#include <cmath>

class CmdVelBridgeNode : public rclcpp::Node
{
public:
    CmdVelBridgeNode()
    : rclcpp::Node("igvc_cmd_vel_bridge")
    {
        // ── Parameters ────────────────────────────────────────────────────
        declare_parameter<double>("wheel_base",           0.508);
        declare_parameter<double>("max_linear_speed",     1.0);
        declare_parameter<bool>  ("right_motor_inverted", true);
        declare_parameter<bool>  ("left_motor_inverted",  false);
        declare_parameter<double>("cmd_timeout",          0.5);

        wheel_base_           = get_parameter("wheel_base").as_double();
        max_linear_speed_     = get_parameter("max_linear_speed").as_double();
        right_motor_inverted_ = get_parameter("right_motor_inverted").as_bool();
        left_motor_inverted_  = get_parameter("left_motor_inverted").as_bool();
        cmd_timeout_          = get_parameter("cmd_timeout").as_double();

        // ── Publishers / Subscribers ──────────────────────────────────────
        pub_motor_cmd_ = create_publisher<std_msgs::msg::String>("/igvc/motor_cmd", 10);

        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelBridgeNode::cmdVelCb, this, std::placeholders::_1));

        // Watchdog timer at 10 Hz
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CmdVelBridgeNode::watchdogCb, this));

        last_cmd_time_ = now();

        RCLCPP_INFO(get_logger(),
            "cmd_vel_bridge ready  wheel_base=%.3f m  max_speed=%.2f m/s",
            wheel_base_, max_linear_speed_);
    }

private:
    // ── Callbacks ─────────────────────────────────────────────────────────
    void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = now();

        const double vx = msg->linear.x;
        const double wz = msg->angular.z;

        // Differential drive kinematics
        const double v_left  = vx - wz * (wheel_base_ / 2.0);
        const double v_right = vx + wz * (wheel_base_ / 2.0);

        // Normalise to duty cycle [-1, 1]
        double duty_left  = clamp(v_left  / max_linear_speed_, -1.0, 1.0);
        double duty_right = clamp(v_right / max_linear_speed_, -1.0, 1.0);

        // Apply per-motor inversion (must match serial_bridge_params / teleop_params)
        const double l = left_motor_inverted_  ? -duty_left  : duty_left;
        const double r = right_motor_inverted_ ? -duty_right : duty_right;

        char cmd[64];
        snprintf(cmd, sizeof(cmd), "M %.4f %.4f\n", l, r);
        publishCmd(std::string(cmd));
    }

    void watchdogCb()
    {
        const double elapsed = (now() - last_cmd_time_).seconds();
        if (elapsed > cmd_timeout_) {
            publishCmd("S\n");
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────
    void publishCmd(const std::string & cmd)
    {
        std_msgs::msg::String msg;
        msg.data = cmd;
        pub_motor_cmd_->publish(msg);
    }

    static double clamp(double v, double lo, double hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    // ── Members ───────────────────────────────────────────────────────────
    double wheel_base_           = 0.508;
    double max_linear_speed_     = 1.0;
    bool   right_motor_inverted_ = true;
    bool   left_motor_inverted_  = false;
    double cmd_timeout_          = 0.5;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        pub_motor_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr                               watchdog_timer_;
    rclcpp::Time                                               last_cmd_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
