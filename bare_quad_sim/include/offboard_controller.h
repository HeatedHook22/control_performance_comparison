#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <memory>

#include "attitude_rate_control.h"

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
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    // subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;

    // Controller objects
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

    // Physical Parameters
    Eigen::Vector3d _g;
    double _m;

    // Position & Velocity
    Eigen::Vector3d _p, _v, _a, _pd, _vd, _ad, _Kp, _Kv, _a_cmd, _u;
    Eigen::Vector3d _v_int, _Kvi, _v_int_limit;
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition &msg);
    void publish_position_setpoint();
    void publish_velocity_setpoint();
    void publish_acceleration_setpoint();
};