#include "attitude_rate_control.h"

/* ============================== Attitude ============================== */
attitude_rates::attitude_rates() { _Katt << 6.5, 6.5, 2.8; }

void attitude_rates::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude &msg) {
    _q(0) = msg.q[0];
    _q(1) = msg.q[1];
    _q(2) = msg.q[2];
    _q(3) = msg.q[3];
}

Eigen::Vector4d attitude_rates::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

    zb_des = -vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    // RCLCPP_INFO(this->get_logger(), "proj_xb_des: %f, %f, %f", proj_xb_des(0), proj_xb_des(1), proj_xb_des(2));
    // RCLCPP_INFO(this->get_logger(), "zb_des: %f, %f, %f", zb_des(0), zb_des(1), zb_des(2));
    // RCLCPP_INFO(this->get_logger(), "yb_des: %f, %f, %f", yb_des(0), yb_des(1), yb_des(2));
    // RCLCPP_INFO(this->get_logger(), "xb_des: %f, %f, %f", xb_des(0), xb_des(1), xb_des(2));

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);

    return quat;
}

inline Eigen::Vector4d attitude_rates::rot2Quaternion(const Eigen::Matrix3d &R) {
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

Eigen::Vector3d attitude_rates::compute_rates_setpoint(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &K_att) {
    Eigen::Vector3d des_rate;
    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);
    des_rate(0) = 2 * K_att(0) * std::copysign(1.0, qe(0)) * qe(1);
    des_rate(1) = 2 * K_att(1) * std::copysign(1.0, qe(0)) * qe(2);
    des_rate(2) = 2 * K_att(2) * std::copysign(1.0, qe(0)) * qe(3);
    return des_rate;
}

inline Eigen::Vector4d attitude_rates::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
    Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}

inline Eigen::Matrix3d attitude_rates::quat2RotMatrix(const Eigen::Vector4d &q) {
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3), 2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3), 2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3), q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}

void attitude_rates::publish_attitude_setpoint(uint64_t timestamp) {
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    Eigen::Vector4f _qdf = _qd.cast<float>();
    msg.q_d = {_qdf(0), _qdf(1), _qdf(2), _qdf(3)};
    msg.thrust_body = {0.0, 0.0, _thrustdn};

    msg.timestamp = timestamp;
    vehicle_attitude_setpoint_publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Attitude command send");
}

void attitude_rates::publish_rates_setpoint(uint64_t timestamp) {
    px4_msgs::msg::VehicleRatesSetpoint msg{};
    Eigen::Vector3f _omegadf = _omegad.cast<float>();
    msg.roll = _omegadf(0);
    msg.pitch = _omegadf(1);
    msg.yaw = _omegadf(2);
    msg.thrust_body = {0.0, 0.0, _thrustdn};

    msg.timestamp = timestamp;
    vehicle_rates_setpoint_publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Angular rates command send");
}
