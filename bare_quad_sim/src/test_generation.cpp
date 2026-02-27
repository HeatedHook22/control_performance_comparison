#include "test_generation.h"
#include "pos_vel_acc_control.h"

test_generation::test_generation(pos_vel_acc &pos_vel_acc_ctrl, attitude_rates &att_rate_ctrl)
    : pos_vel_acc_ctrl(pos_vel_acc_ctrl), att_rate_ctrl(att_rate_ctrl) {}

bool test_generation::is_at_setpoint() {
    bool at_setpoint = false;

    auto tmp_ep = pos_vel_acc_ctrl._pd - pos_vel_acc_ctrl._p;
    auto tmp_ev = pos_vel_acc_ctrl._vd - pos_vel_acc_ctrl._v;

    if ((std::abs(tmp_ep(0)) < 5e-2) && (std::abs(tmp_ep(1)) < 5e-2) && (std::abs(tmp_ep(2)) < 5e-2) && (std::abs(tmp_ev(0)) < 5e-2) &&
        (std::abs(tmp_ev(1)) < 5e-2) && (std::abs(tmp_ev(2)) < 5e-2)) {
        at_setpoint = true;
    }

    return at_setpoint;
}

void test_generation::step_pos_setpoint(const Eigen::Vector3d &pos_sp) { pos_vel_acc_ctrl._pd = pos_sp; }

void test_generation::step_roll_setpoint(const Eigen::Vector3d &roll_sp) { att_rate_ctrl._qd = att_rate_ctrl.euler2quaternion(roll_sp); }
void test_generation::step_pitch_setpoint(const Eigen::Vector3d &pitch_sp) { att_rate_ctrl._qd = att_rate_ctrl.euler2quaternion(pitch_sp); }
void test_generation::step_yaw_setpoint(const Eigen::Vector3d &yaw_sp) { att_rate_ctrl._qd = att_rate_ctrl.euler2quaternion(yaw_sp); }