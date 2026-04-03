#pragma once

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace igvc_control
{

class IgvcHardware : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    void openSerial();
    void closeSerial();
    void writeSerial(const std::string & data);
    int  readSerial(char * buf, size_t len);

    // Serial
    std::string serial_port_;
    int         serial_fd_ = -1;

    // Drivetrain constants
    double gear_ratio_     = 12.75;
    double max_motor_rpm_  = 5700.0;
    bool   left_inverted_  = false;
    bool   right_inverted_ = true;

    // Joint state: [0] = left, [1] = right
    double hw_positions_[2] = {0.0, 0.0};   // radians (integrated)
    double hw_velocities_[2] = {0.0, 0.0};  // rad/s (from encoder)
    double hw_commands_[2] = {0.0, 0.0};     // rad/s (from controller)

    // Serial RX line buffer
    char   rx_buf_[256];
    int    rx_len_ = 0;

    rclcpp::Logger logger_ = rclcpp::get_logger("IgvcHardware");
};

}  // namespace igvc_control
