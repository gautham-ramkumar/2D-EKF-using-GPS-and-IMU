#pragma once
#include <Eigen/Dense>

using Vec5 = Eigen::Matrix<double, 5, 1>;   // 5 numbers in a column (the state)
using Mat5 = Eigen::Matrix<double, 5, 5>;   // 5×5 matrix (P, Qc, F)
using Mat2 = Eigen::Matrix<double, 2, 2>;   // 2×2 matrix (R)

class Ekf2D {
public:
    Ekf2D();
    void setInitialState(double px, double py, double theta, double v);
    void predict(double a, double omega, double dt);
    // GPS update. Returns false if the fix was gated out (rejected as an outlier).
    bool update(double east, double north);

    // Wheel-speed update: measures v directly (H = [0 0 0 1 0]).
    void updateSpeed(double v);

    const Vec5& state() const { return x_; }       // read-only access
    const Mat5& covariance() const { return P_; }
    double nis() const { return nis_; }            // NIS of the last update (ideal mean: 2)
    int numRejected() const { return nRejected_; }
    int numRecoveries() const { return nRecoveries_; }

private:
    // Gating: reject a fix if its NIS exceeds the 99% chi-square value for
    // 2 degrees of freedom. After maxRejects_ rejects in a row, assume the
    // filter (not the GPS) is wrong and accept the fix anyway.
    static constexpr double kChi2_2dof_99 = 9.21034;
    static constexpr int maxRejects_ = 10;
    int rejectsInRow_ = 0;
    int nRejected_ = 0;
    int nRecoveries_ = 0;
    double nis_ = 0.0;

    Vec5 x_;     // [px, py, theta, v, b_a]
    Mat5 P_;     // state covariance
    Mat5 Qc_;    // process noise (per second)
    Mat2 R_;     // GPS noise
    double rSpeed_ = 0.2 * 0.2;   // wheel-speed noise (m/s)²
};