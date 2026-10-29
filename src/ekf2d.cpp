#include "ekf2d.hpp"
#include <cmath>

// Wrap an angle to [-pi, pi], like Python's wrap().
static double wrap(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

Ekf2D::Ekf2D() {

    const double theta0Std = 8.0 * M_PI / 180.0;   // 8° in radians
    const double theta0Var = theta0Std * theta0Std;

    x_.setZero();
    P_.setZero();
    P_.diagonal() << 1.0, 1.0, theta0Var, 1.0, 0.0;
    Qc_.setZero();
    Qc_.diagonal() << 0.2, 0.2, 1e-3, 0.2, 0.0;
    R_.setIdentity();
    R_ *= 0.25;

}

void Ekf2D::setInitialState(double px, double py, double theta, double v) {
    x_(0) = px;
    x_(1) = py;
    x_(2) = theta;
    x_(3) = v;
    x_(4) = 0.0;
}

void Ekf2D::predict(double a, double omega, double dt) {
    const double px = x_(0);
    const double py = x_(1);
    const double theta = x_(2);
    const double v = x_(3);
    const double b_a = x_(4);

    // Predict the new state
    x_(0) = px + v * std::cos(theta) * dt;
    x_(1) = py + v * std::sin(theta) * dt;
    x_(2) = wrap(theta + omega * dt);
    x_(3) = v + (a - b_a) * dt;
    x_(4) = b_a;

    // Predict the new covariance
    Mat5 F = Mat5::Identity();
    F(0, 2) = -v * std::sin(theta) * dt;
    F(0, 3) = std::cos(theta) * dt;
    F(1, 2) = v * std::cos(theta) * dt;
    F(1, 3) = std::sin(theta) * dt;
    F(3, 4) = -dt;

    // Update P (covariance) after prediction
    P_ = F * P_ * F.transpose() + Qc_ * dt;

}

bool Ekf2D::update(double east, double north) {
    // H picks px, py out of the 5-state
    Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Vector2d z(east, north);

    Eigen::Vector2d y = z - H * x_;                  // Python: y = z - H @ self.x
    Mat2 S = H * P_ * H.transpose() + R_;            // Python: S = H @ self.P @ H.T + self.R
    Mat2 S_inv = S.inverse();
    nis_ = (y.transpose() * S_inv * y).value();      // Python: self.nis = y @ S_inv @ y  (1×1 → double)

    if (nis_ > kChi2_2dof_99) {
        // Multipath jump, or the filter itself has drifted. After 1 s of
        // rejected fixes, assume the filter is wrong: grow the position
        // covariance by the innovation and take the fix.
        rejectsInRow_++;
        if (rejectsInRow_ < maxRejects_) {
            nRejected_++;
            return false;
        }
        P_.topLeftCorner<2, 2>() += y * y.transpose();
        S = H * P_ * H.transpose() + R_;
        S_inv = S.inverse();
        nRecoveries_++;
    }
    rejectsInRow_ = 0;

    Eigen::Matrix<double, 5, 2> K = P_ * H.transpose() * S_inv;            // Python: K = self.P @ H.T @ S_inv

    x_ = x_ + K * y;                                 // Python: self.x = self.x + K @ y
    x_(2) = wrap(x_(2));

    Mat5 I_KH = Mat5::Identity() - K * H;            // Python: I_KH = np.eye(5) - K @ H
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();  // Python: I_KH @ P @ I_KH.T + K @ R @ K.T
    return true;
}

void Ekf2D::updateSpeed(double v) {
    // Same steps as the GPS update, but the measurement is one number, so
    // H is 1×5 and S is a plain double: no matrix inverse needed.
    Eigen::Matrix<double, 1, 5> H = Eigen::Matrix<double, 1, 5>::Zero();
    H(0, 3) = 1.0;

    const double y = v - x_(3);                                // innovation
    const double S = (H * P_ * H.transpose()).value() + rSpeed_;
    const Vec5 K = P_ * H.transpose() / S;                     // 5×1 gain

    x_ = x_ + K * y;
    x_(2) = wrap(x_(2));

    const Mat5 I_KH = Mat5::Identity() - K * H;                // Joseph form, as in update()
    P_ = I_KH * P_ * I_KH.transpose() + K * rSpeed_ * K.transpose();
}
