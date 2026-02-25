#pragma once

#include <chrono>
#include <stdint.h>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Eigen>

class attitude_rates {
  public:
    attitude_rates();

    // Attitude & Angular Rate Thrust
    Eigen::Vector4d _q, _qd;
    Eigen::Vector3d _omega, _omegad, _Katt;
    float _thrustd, _thrustdn; // thrust in force, normalized thrust
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude &msg);
    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
    inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R);
    Eigen::Vector3d compute_rates_setpoint(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &K_att);
    inline Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
    inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q);

    void publish_attitude_setpoint(uint64_t timestamp);
    void publish_rates_setpoint(uint64_t timestamp);

    // PUB
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;

    // SUB
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;

  private:
};