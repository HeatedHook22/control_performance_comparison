/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

// My includes
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <memory>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class OffboardControl : public rclcpp::Node {
  public:
    OffboardControl() : Node("offboard_control") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        vehicle_attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", 10);
        vehicle_rates_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);

        vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos, std::bind(&OffboardControl::vehicle_local_position_callback, this, _1));
        vehicle_attitude_subscription_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
                                                                                    std::bind(&OffboardControl::vehicle_attitude_callback, this, _1));

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
            }

            _last_time = _current_time;
            _current_time = this->now();
            RCLCPP_INFO(this->get_logger(), "ct-lt: %f, dt: %f", _current_time.seconds() - _last_time.seconds(), _dt);

            update_control();

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();

            if (_enable_position_cmd)
                publish_position_setpoint();
            else if (_enable_velocity_cmd)
                publish_velocity_setpoint();
            else if (_enable_acceleration_cmd)
                publish_acceleration_setpoint();
            else if (_enable_attitude_cmd)
                publish_attitude_setpoint();
            else if (_enable_rate_cmd)
                publish_rates_setpoint();

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);

        _g << 0.0, 0.0, 9.80665;
        _m = 2.064;
        _dt = 0.1;
        _pd << 0.0, 0.0, -5.0;
        _vd << 0.0, 0.0, 0.0;
        _ad << 0.0, 0.0, 0.0;
        _Kp << 5, 5, 10;
        _Kv << 2, 2, 4;
        _Kvi << 0.4, 0.4, 2;
        _v_int_limit << 5.0, 5.0, 5.0;
        _Katt << 6.5, 6.5, 2.8;
    }

    void arm();
    void disarm();

  private:
    // Offboard Mode
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;    //!< common synced timestamped
    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    // publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;
    // subscribers
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;

    // control mode
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
    void vehicle_local_position_callback(const VehicleLocalPosition &msg);
    void publish_position_setpoint();
    void publish_velocity_setpoint();
    void publish_acceleration_setpoint();

    // Attitude & Angular Rate Thrust
    Eigen::Vector4d _q, _qd;
    Eigen::Vector3d _omega, _omegad, _Katt;
    double _yawd = M_PI * 0.5;
    float _thrustd, _thrustdn; // thrust in force, normalized thrust
    void vehicle_attitude_callback(const VehicleAttitude &msg);
    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
    inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R);
    Eigen::Vector3d compute_rates_setpoint(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &K_att);
    inline Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
    inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q);
    void publish_attitude_setpoint();
    void publish_rates_setpoint();
};

int main(int argc, char *argv[]) {
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = _enable_position_cmd;
    msg.velocity = _enable_velocity_cmd;
    msg.acceleration = _enable_acceleration_cmd;
    msg.attitude = _enable_attitude_cmd;
    msg.body_rate = _enable_rate_cmd;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

/* ============================== Position / Velocity ============================== */

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_position_setpoint() {
    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = _yawd;
    msg.yawspeed = NAN;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Position command send");
}

void OffboardControl::publish_velocity_setpoint() {
    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    Eigen::Vector3f epf = (_pd - _p).cast<float>();
    msg.velocity = {epf(0), epf(1), epf(2)};
    msg.yaw = _yawd;
    msg.yawspeed = NAN;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Velocity command send");
}

void OffboardControl::publish_acceleration_setpoint() {
    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {NAN, NAN, NAN};
    Eigen::Vector3f _a_cmdf = _a_cmd.cast<float>();
    msg.acceleration = {_a_cmdf.x(), _a_cmdf.y(), _a_cmdf.z()};
    msg.yaw = _yawd;
    msg.yawspeed = NAN;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Acceleration command send");
}

void OffboardControl::vehicle_local_position_callback(const VehicleLocalPosition &msg) {
    // RCLCPP_INFO(this->get_logger(), "position: %f", msg.x);
    _p.x() = msg.x;
    _p.y() = msg.y;
    _p.z() = msg.z;
    _v.x() = msg.vx;
    _v.y() = msg.vy;
    _v.z() = msg.vz;
    _a.x() = msg.ax;
    _a.y() = msg.ay;
    _a.z() = msg.az;
}

/* ============================== Attitude ============================== */

void OffboardControl::vehicle_attitude_callback(const VehicleAttitude &msg) {
    // RCLCPP_INFO(this->get_logger(), "attitude: %f", msg.q[0]);
    _q(0) = msg.q[0];
    _q(1) = msg.q[1];
    _q(2) = msg.q[2];
    _q(3) = msg.q[3];
}

void OffboardControl::update_control() {
    // p, v,  and errors
    RCLCPP_INFO(this->get_logger(), "p: %f, %f, %f", _p(0), _p(1), _p(2));
    RCLCPP_INFO(this->get_logger(), "v: %f, %f, %f", _v(0), _v(1), _v(2));

    _ep = _pd - _p;
    _ev = _vd - _v;

    RCLCPP_INFO(this->get_logger(), "ep: %f, %f, %f", _ep(0), _ep(1), _ep(2));
    RCLCPP_INFO(this->get_logger(), "ev: %f, %f, %f", _ev(0), _ev(1), _ev(2));

    // Integration of velocity errors
    if (_enable_velocity_integrator) {
        for (int i = 0; i < 3; i++) {
            if (std::abs(_v_int(i) + (_ev(i)) * _dt) <= _v_int_limit(i))
                _v_int(i) += _ev(i) * _dt;
        }
    }
    RCLCPP_INFO(this->get_logger(), "_v_int: %f, %f, %f", _v_int.x(), _v_int.y(), _v_int.z());

    // Auxiliary input
    _a_cmd = _ad + _Kv.cwiseProduct(_ev) + _Kp.cwiseProduct(_ep) + _Kvi.cwiseProduct(_v_int);
    RCLCPP_INFO(this->get_logger(), "_a_cmd: %f, %f, %f", _a_cmd.x(), _a_cmd.y(), _a_cmd.z());

    // Gravity compensation
    _u = _a_cmd - _g;
    RCLCPP_INFO(this->get_logger(), "_u: %f, %f, %f", _u.x(), _u.y(), _u.z());

    // Convert acceleration _u to desired attitudes _qd
    _qd = acc2quaternion(_u, _yawd);
    RCLCPP_INFO(this->get_logger(), "_qd: %f, %f, %f, %f", _qd(0), _qd.x(), _qd.y(), _qd.z());

    // Thrust command
    _thrustd = -_u.norm();
    RCLCPP_INFO(this->get_logger(), "_thrustd: %f", _thrustd);

    // Normalized thrust
    _thrustdn = std::max(std::min(0.0, 0.07421 * _thrustd), -1.0);
    RCLCPP_INFO(this->get_logger(), "_thrustdn: %f", _thrustdn);

    // Angular Rate
    _omegad = compute_rates_setpoint(_q, _qd, _Katt);
    RCLCPP_INFO(this->get_logger(), "_omegad: %f, %f, %f", _omegad.x(), _omegad.y(), _omegad.z());
}

Eigen::Vector4d OffboardControl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

    zb_des = -vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    RCLCPP_INFO(this->get_logger(), "proj_xb_des: %f, %f, %f", proj_xb_des(0), proj_xb_des(1), proj_xb_des(2));
    RCLCPP_INFO(this->get_logger(), "zb_des: %f, %f, %f", zb_des(0), zb_des(1), zb_des(2));
    RCLCPP_INFO(this->get_logger(), "yb_des: %f, %f, %f", yb_des(0), yb_des(1), yb_des(2));
    RCLCPP_INFO(this->get_logger(), "xb_des: %f, %f, %f", xb_des(0), xb_des(1), xb_des(2));

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);

    return quat;
}

inline Eigen::Vector4d OffboardControl::rot2Quaternion(const Eigen::Matrix3d &R) {
    Eigen::Vector4d quat;
    double tr = R.trace();
    if (tr > 0.0) {
        double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
        quat(0) = 0.25 * S;
        quat(1) = (R(2, 1) - R(1, 2)) / S;
        quat(2) = (R(0, 2) - R(2, 0)) / S;
        quat(3) = (R(1, 0) - R(0, 1)) / S;
    } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
        quat(0) = (R(2, 1) - R(1, 2)) / S;
        quat(1) = 0.25 * S;
        quat(2) = (R(0, 1) + R(1, 0)) / S;
        quat(3) = (R(0, 2) + R(2, 0)) / S;
    } else if (R(1, 1) > R(2, 2)) {
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
        quat(0) = (R(0, 2) - R(2, 0)) / S;
        quat(1) = (R(0, 1) + R(1, 0)) / S;
        quat(2) = 0.25 * S;
        quat(3) = (R(1, 2) + R(2, 1)) / S;
    } else {
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
        quat(0) = (R(1, 0) - R(0, 1)) / S;
        quat(1) = (R(0, 2) + R(2, 0)) / S;
        quat(2) = (R(1, 2) + R(2, 1)) / S;
        quat(3) = 0.25 * S;
    }
    return quat;
}

Eigen::Vector3d OffboardControl::compute_rates_setpoint(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &K_att) {
    Eigen::Vector3d des_rate;
    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);
    des_rate(0) = 2 * K_att(0) * std::copysign(1.0, qe(0)) * qe(1);
    des_rate(1) = 2 * K_att(1) * std::copysign(1.0, qe(0)) * qe(2);
    des_rate(2) = 2 * K_att(2) * std::copysign(1.0, qe(0)) * qe(3);
    return des_rate;
}

inline Eigen::Vector4d OffboardControl::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
    Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}

inline Eigen::Matrix3d OffboardControl::quat2RotMatrix(const Eigen::Vector4d &q) {
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3), 2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3), 2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3), q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}

void OffboardControl::publish_attitude_setpoint() {
    VehicleAttitudeSetpoint msg{};
    Eigen::Vector4f _qdf = _qd.cast<float>();
    msg.q_d = {_qdf(0), _qdf(1), _qdf(2), _qdf(3)};
    msg.thrust_body = {0.0, 0.0, _thrustdn};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_attitude_setpoint_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Attitude command send");
}

void OffboardControl::publish_rates_setpoint() {
    VehicleRatesSetpoint msg{};
    Eigen::Vector3f _omegadf = _omegad.cast<float>();
    msg.roll = _omegadf(0);
    msg.pitch = _omegadf(1);
    msg.yaw = _omegadf(2);
    msg.thrust_body = {0.0, 0.0, _thrustdn};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_rates_setpoint_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Angular rates command send");
}
