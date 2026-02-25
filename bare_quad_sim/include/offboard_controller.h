#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <memory>

#include "attitude_rate_control.h"
#include "pos_vel_acc_control.h"

class OffboardControl : public rclcpp::Node {
  public:
    OffboardControl();
    void arm();
    void disarm();

  private:
    // Offboard Mode
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp;     //!< common synced timestamped
    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Controller objects
    pos_vel_acc pos_vel_acc_ctrl{};
    attitude_rates att_rate_ctrl{};

    // Control mode
    bool _enable_position_cmd = false;
    bool _enable_velocity_cmd = false;
    bool _enable_acceleration_cmd = false;
    bool _enable_velocity_integrator = false;
    bool _enable_attitude_cmd = false;
    bool _enable_rate_cmd = true;

    // Control
    void update_control();
    rclcpp::Time _current_time, _last_time;
    double _dt;
    Eigen::Vector3d _ep, _ev;

    // Control Setpoint Commands
    uint64_t current_setpoint_step = 0;
    void set_setpoint();
    void step_setpoint(const Eigen::Vector3d &pos_sp);
    bool is_at_setpoint();

    // Physical Parameters
    Eigen::Vector3d _g;
    double _m;
};