#pragma once

#include <eigen3/Eigen/Eigen>

#include "attitude_rate_control.h"
#include "pos_vel_acc_control.h"

class test_generation {
  public:
    test_generation(pos_vel_acc &pos_vel_acc_ctrl, attitude_rates &att_rate_ctrl);

    bool is_at_setpoint();
    void step_pos_setpoint(const Eigen::Vector3d &pos_sp);
    void step_rpy_setpoint(const Eigen::Vector3d &euler_sp);
    void sinusoid_rpy_setpoint(const Eigen::Vector3d &euler_sp_amplitude, double frequency);

  private:
    pos_vel_acc &pos_vel_acc_ctrl;
    attitude_rates &att_rate_ctrl;
};