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

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "offboard_controller.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

int main(int argc, char *argv[]) {
    // MUST BE RUN FROM WITHIN PARENT DIRECTORY
    std::cout << "Starting recording script..." << std::endl;
    std::system("python3 data_recording/position_plotter.py &");
    std::this_thread::sleep_for(2000ms);

    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}

OffboardControl::OffboardControl() : Node("offboard_control") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Publishers
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    pos_vel_acc_ctrl.trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    att_rate_ctrl.vehicle_attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", 10);
    att_rate_ctrl.vehicle_rates_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);

    // Subscribers
    pos_vel_acc_ctrl.vehicle_local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", qos, [this](const px4_msgs::msg::VehicleLocalPosition &msg) {
            // RCLCPP_INFO(this->get_logger(), "position: %f", msg.x);
            pos_vel_acc_ctrl.vehicle_local_position_callback(msg);
        });
    att_rate_ctrl.vehicle_attitude_subscription_ =
        this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos, [this](const px4_msgs::msg::VehicleAttitude &msg) {
            // RCLCPP_INFO(this->get_logger(), "attitude: %f", msg.q[0]);
            att_rate_ctrl.vehicle_attitude_callback(msg);
        });

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
        publish_offboard_control_mode();

        timestamp = this->get_clock()->now().nanoseconds() / 1000;
        if (_enable_position_cmd)
            pos_vel_acc_ctrl.publish_position_setpoint(timestamp);
        else if (_enable_velocity_cmd)
            pos_vel_acc_ctrl.publish_velocity_setpoint(timestamp);
        else if (_enable_acceleration_cmd)
            pos_vel_acc_ctrl.publish_acceleration_setpoint(timestamp);
        else if (_enable_attitude_cmd)
            att_rate_ctrl.publish_attitude_setpoint(timestamp);
        else if (_enable_rate_cmd)
            att_rate_ctrl.publish_rates_setpoint(timestamp);

        // stop the counter after reaching 11
        if (offboard_setpoint_counter_ < 110) {
            offboard_setpoint_counter_++;
        }
    };
    timer_ = this->create_wall_timer(10ms, timer_callback);

    // Set Physical Parameters
    _g << 0.0, 0.0, 9.80665;
    _m = 2.064;
    _dt = 0.01;
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
    // Overrides need to land (21196.0), causing instant disarm, for TESTING IN SIM ONLY
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send, shutting down");

    // Early shutdown
    rclcpp::shutdown();
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

void OffboardControl::update_control() {
    // Get setpoint
    set_setpoint();

    // p, v,  and errors
    auto &_p = pos_vel_acc_ctrl._p;
    auto &_v = pos_vel_acc_ctrl._v;

    RCLCPP_INFO(this->get_logger(), "p: %f, %f, %f", _p(0), _p(1), _p(2));
    RCLCPP_INFO(this->get_logger(), "v: %f, %f, %f", _v(0), _v(1), _v(2));

    _ep = pos_vel_acc_ctrl._pd - _p;
    _ev = pos_vel_acc_ctrl._vd - _v;

    RCLCPP_INFO(this->get_logger(), "ep: %f, %f, %f", _ep(0), _ep(1), _ep(2));
    RCLCPP_INFO(this->get_logger(), "ev: %f, %f, %f", _ev(0), _ev(1), _ev(2));

    // Integration of velocity errors
    auto &_v_int = pos_vel_acc_ctrl._v_int;

    if (_enable_velocity_integrator) {
        for (int i = 0; i < 3; i++) {
            if (std::abs(_v_int(i) + (_ev(i)) * _dt) <= pos_vel_acc_ctrl._v_int_limit(i))
                _v_int(i) += _ev(i) * _dt;
        }
    }
    RCLCPP_INFO(this->get_logger(), "_v_int: %f, %f, %f", _v_int.x(), _v_int.y(), _v_int.z());

    // Auxiliary input
    auto &_ad = pos_vel_acc_ctrl._ad;
    auto &_a_cmd = pos_vel_acc_ctrl._a_cmd;
    auto &_Kv = pos_vel_acc_ctrl._Kv;
    auto &_Kp = pos_vel_acc_ctrl._Kp;
    auto &_Kvi = pos_vel_acc_ctrl._Kvi;

    _a_cmd = _ad + _Kv.cwiseProduct(_ev) + _Kp.cwiseProduct(_ep) + _Kvi.cwiseProduct(pos_vel_acc_ctrl._v_int);
    RCLCPP_INFO(this->get_logger(), "_a_cmd: %f, %f, %f", _a_cmd.x(), _a_cmd.y(), _a_cmd.z());

    // Gravity compensation
    auto &_u = pos_vel_acc_ctrl._u;

    _u = pos_vel_acc_ctrl._a_cmd - _g;
    RCLCPP_INFO(this->get_logger(), "_u: %f, %f, %f", _u.x(), _u.y(), _u.z());

    // Convert acceleration _u to desired attitudes _qd
    auto &_qd = att_rate_ctrl._qd;
    auto &_thrustd = att_rate_ctrl._thrustd;
    auto &_thrustdn = att_rate_ctrl._thrustdn;
    auto &_omegad = att_rate_ctrl._omegad;

    _qd = att_rate_ctrl.acc2quaternion(_u, pos_vel_acc_ctrl._yawd);
    RCLCPP_INFO(this->get_logger(), "_qd: %f, %f, %f, %f", _qd(0), _qd.x(), _qd.y(), _qd.z());

    // Thrust command
    _thrustd = -_u.norm();
    RCLCPP_INFO(this->get_logger(), "_thrustd: %f", _thrustd);

    // Normalized thrust
    _thrustdn = std::max(std::min(0.0, 0.07421 * _thrustd), -1.0);
    RCLCPP_INFO(this->get_logger(), "_thrustdn: %f", _thrustdn);

    // Angular Rate
    att_rate_ctrl._omegad = att_rate_ctrl.compute_rates_setpoint(att_rate_ctrl._q, _qd, att_rate_ctrl._Katt);
    RCLCPP_INFO(this->get_logger(), "_omegad: %f, %f, %f", _omegad.x(), _omegad.y(), _omegad.z());
}

void OffboardControl::set_setpoint() {
    switch (current_setpoint_step) {
    case 0: {
        Eigen::Vector3d pos_sp(1.f, 2.f, -6.f);
        step_setpoint(pos_sp);
        break;
    }
    case 1: {
        Eigen::Vector3d pos_sp(5.f, 8.f, -6.f);
        step_setpoint(pos_sp);
        break;
    }
    default: {
        // Change modes for landing (smoother for exiting test cases)
        _enable_position_cmd = true;
        _enable_velocity_cmd = false;
        _enable_acceleration_cmd = false;
        _enable_velocity_integrator = false;
        _enable_attitude_cmd = false;
        _enable_rate_cmd = false;

        // Land vehicle
        Eigen::Vector3d pos_sp(0.f, 0.f, -1.f);
        step_setpoint(pos_sp);

        if (is_at_setpoint()) {
            RCLCPP_INFO(this->get_logger(), "DISARMING");
            disarm();
        }
        break;
    }
    }

    if (is_at_setpoint()) {
        current_setpoint_step++;
    }
}

bool OffboardControl::is_at_setpoint() {
    bool at_setpoint = false;

    auto tmp_ep = pos_vel_acc_ctrl._pd - pos_vel_acc_ctrl._p;
    auto tmp_ev = pos_vel_acc_ctrl._vd - pos_vel_acc_ctrl._v;

    if ((std::abs(tmp_ep(0)) < 5e-2) && (std::abs(tmp_ep(1)) < 5e-2) && (std::abs(tmp_ep(2)) < 5e-2) && (std::abs(tmp_ev(0)) < 5e-2) &&
        (std::abs(tmp_ev(1)) < 5e-2) && (std::abs(tmp_ev(2)) < 5e-2)) {
        at_setpoint = true;
    }

    RCLCPP_INFO(this->get_logger(), "current_setpoint_step: %ld, tmp_ep: %f, %f, %f, tmp_ev: %f, %f, %f", current_setpoint_step, tmp_ep(0), tmp_ep(1),
                tmp_ep(2), tmp_ev(0), tmp_ev(1), tmp_ev(2));
    return at_setpoint;
}

void OffboardControl::step_setpoint(const Eigen::Vector3d &pos_sp) { pos_vel_acc_ctrl._pd = pos_sp; }
