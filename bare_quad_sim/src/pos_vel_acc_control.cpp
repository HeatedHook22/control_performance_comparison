#include "pos_vel_acc_control.h"

/* ============================== Position / Velocity ============================== */
pos_vel_acc::pos_vel_acc() {
    _pd << 0.0, 0.0, -1.0;
    _vd << 0.0, 0.0, 0.0;
    _ad << 0.0, 0.0, 0.0;
    _Kp << 5, 5, 10;
    _Kv << 2, 2, 4;
    _Kvi << 0.4, 0.4, 2;
    _v_int_limit << 5.0, 5.0, 5.0;
}
/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void pos_vel_acc::publish_position_setpoint(uint64_t timestamp) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {_pd.cast<float>()(0), _pd.cast<float>()(1), _pd.cast<float>()(2)};
    msg.yaw = _yawd;
    msg.yawspeed = NAN;
    msg.timestamp = timestamp;
    trajectory_setpoint_publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Position command send");
}

void pos_vel_acc::publish_velocity_setpoint(uint64_t timestamp) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    Eigen::Vector3f epf = (_pd - _p).cast<float>();
    msg.velocity = {epf(0), epf(1), epf(2)};
    msg.yaw = _yawd;
    msg.yawspeed = NAN;
    msg.timestamp = timestamp;
    trajectory_setpoint_publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Velocity command send");
}

void pos_vel_acc::publish_acceleration_setpoint(uint64_t timestamp) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {NAN, NAN, NAN};
    Eigen::Vector3f _a_cmdf = _a_cmd.cast<float>();
    msg.acceleration = {_a_cmdf.x(), _a_cmdf.y(), _a_cmdf.z()};
    msg.yaw = _yawd;
    msg.yawspeed = NAN;
    msg.timestamp = timestamp;
    trajectory_setpoint_publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Acceleration command send");
}

void pos_vel_acc::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition &msg) {
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