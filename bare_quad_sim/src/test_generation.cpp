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

void test_generation::sinusoid_setpoint(const Eigen::Vector3d &pos_sp, double frequency) {
    static auto start_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    double time_sec = std::chrono::duration<double>(current_time - start_time).count();

    auto pos_sp_sin = pos_sp * std::sin(time_sec * frequency * 2 * M_PI); // Sinusoidally vary the position setpoint with the given amplitude
    auto pos_sp_compensated =
        pos_sp_sin + pos_vel_acc_ctrl._p; // Add the sinusoidal variation to the current setpoint to avoid commanding large jumps

    this->step_pos_setpoint(pos_sp_compensated); // Sinusoidally vary the position setpoint with the given amplitude
}

void test_generation::step_rpy_setpoint(const Eigen::Vector3d &euler_sp) { att_rate_ctrl._qd = att_rate_ctrl.euler2quaternion(euler_sp); }

void test_generation::sinusoid_rpy_setpoint(const Eigen::Vector3d &euler_sp_amplitude, double frequency) {
    static auto start_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    double time_sec = std::chrono::duration<double>(current_time - start_time).count();

    Eigen::Vector3d euler_sp_sin = euler_sp_amplitude * std::sin(time_sec * frequency * 2 * M_PI);

    // Add to current yaw to setpoint just to avoid commanding yaw by accident when passing in 0.f
    euler_sp_sin(2) += pos_vel_acc_ctrl._yawd;

    att_rate_ctrl._qd = att_rate_ctrl.euler2quaternion(euler_sp_sin);
}