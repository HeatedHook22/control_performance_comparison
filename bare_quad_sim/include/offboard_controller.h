#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <memory>

#include "attitude_rate_control.h"
#include "pos_vel_acc_control.h"
#include "test_generation.h"

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

    enum offboard_control_mode_enums : uint8_t {
        NONE = 0b0,
        POSITION = 0b1,
        VELOCITY = 0b10,
        ACCELERATION = 0b100,
        ATTITUDE = 0b1000,
        BODY_RATE = 0b10000
    };
    void set_offboard_control_mode(uint8_t offboard_control_mode);

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Controller objects
    pos_vel_acc pos_vel_acc_ctrl{};
    attitude_rates att_rate_ctrl{};

    // Control mode
    bool _enable_position_cmd = true;
    bool _enable_velocity_cmd = false;
    bool _enable_acceleration_cmd = false;
    bool _enable_attitude_cmd = false;
    bool _enable_rate_cmd = false;

    bool _enable_velocity_integrator = false;

    // Control
    void update_control();
    rclcpp::Time _current_time, _last_time;
    double _dt;
    Eigen::Vector3d _ep, _ev;
    bool bypass_update_control = false;

    // Control Setpoint Commands
    void set_setpoint();
    test_generation test_gen{pos_vel_acc_ctrl, att_rate_ctrl};
    uint64_t current_setpoint_step = 0;

    // Test Member Helpers
    // For Constant roll
    const double degrees_sp = 40.0;
    const float radians_sp = static_cast<float>(degrees_sp * M_PI / 180.0);

    // Attitude Test Helpers
    void attitude_step_roll_test();

    // Body Rate Test Helpers
    void body_rate_step_roll_test();

    // Physical Parameters
    Eigen::Vector3d _g;
    double _m;
};