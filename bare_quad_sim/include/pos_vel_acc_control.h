#pragma once

#include <stdint.h>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Eigen>

class pos_vel_acc {
  public:
    pos_vel_acc();

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition &msg);
    void publish_position_setpoint(uint64_t timestamp);
    void publish_velocity_setpoint(uint64_t timestamp);
    void publish_acceleration_setpoint(uint64_t timestamp);

    // PUB
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    // SUB
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;

    double _yawd = M_PI * 0.5;

    // Position & Velocity
    Eigen::Vector3d _p, _v, _a, _pd, _vd, _ad, _Kp, _Kv, _a_cmd, _u;
    Eigen::Vector3d _v_int, _Kvi, _v_int_limit;

  private:
};