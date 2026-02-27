#pragma once

#include <eigen3/Eigen/Eigen>

#include "attitude_rate_control.h"
#include "pos_vel_acc_control.h"

class test_generation {
  public:
    test_generation(pos_vel_acc &pos_vel_acc_ctrl, attitude_rates &att_rate_ctrl);

    bool is_at_setpoint();
    void step_pos_setpoint(const Eigen::Vector3d &pos_sp);
    void step_roll_setpoint(const Eigen::Vector3d &roll_sp);
    void step_pitch_setpoint(const Eigen::Vector3d &pitch_sp);
    void step_yaw_setpoint(const Eigen::Vector3d &yaw_sp);

  private:
    pos_vel_acc &pos_vel_acc_ctrl;
    attitude_rates &att_rate_ctrl;
};