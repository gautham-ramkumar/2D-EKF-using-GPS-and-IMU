// Runs three estimators live on ROS 2 topics (e.g. from `ros2 bag play`) and
// publishes them side by side for RViz:
//   EKF-A       IMU + GPS
//   EKF-B       IMU + wheel speed + GPS   (main estimate: /ekf/odom and TF)
//   wheel odom  wheel speed + gyro only, no GPS (pure dead reckoning)
// plus every GPS fix as a white reference, EKF-B's current 2-sigma
// covariance ellipse, and a trail of past ellipses (outlines, one every
// `ellipse_spacing` meters traveled).
//
// Same filter and preprocessing as the offline main.cpp, made causal:
//   - IMU biases come from the first 2 s of IMU data (car parked).
//   - Everything starts once the car is 15 m from the first GPS fix, with the
//     heading from the first fix to that one. Offline, the start heading uses
//     a fix from the "future", so the live run starts ~15 s later.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigenvalues>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ekf2d.hpp"
#include "utm.hpp"

namespace {

constexpr double kCalibrationTime = 2.0;   // s of parked IMU used for bias
constexpr double kMaxDt = 0.05;            // s, same cap as offline
constexpr double kMaxGpsSpeed = 40.0;      // m/s, teleport rejection
constexpr double kStartDistance = 15.0;    // m from the first fix before starting
constexpr double kPathPeriod = 0.5;        // s between poses appended to the paths
constexpr int kMaxEllipses = 400;          // trail length (oldest are overwritten)
constexpr int kEllipsePoints = 48;         // vertices per ellipse outline
constexpr double kGpsRefPeriod = 1.0;      // s between re-publishing the GPS reference

using Marker = visualization_msgs::msg::Marker;
using Time = builtin_interfaces::msg::Time;

double toSec(const Time& t) {
    return t.sec + t.nanosec * 1e-9;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

std_msgs::msg::ColorRGBA rgba(double r, double g, double b, double a = 1.0) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

// One trajectory shown in RViz: its path and its floating name label.
struct Track {
    std::string name;
    std_msgs::msg::ColorRGBA color;
    double labelOffset;   // m north of the vehicle, so labels don't overlap
    nav_msgs::msg::Path path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub;
};

}  // namespace

class Ekf2DNode : public rclcpp::Node {
public:
    Ekf2DNode() : Node("ekf2d_node") {
        mapFrame_ = declare_parameter("map_frame", std::string("map"));
        baseFrame_ = declare_parameter("base_frame", std::string("base_link"));
        // Marker sizes are in meters, so they shrink on screen as you zoom out.
        // Defaults suit the whole-drive view (about 0.3 px per meter).
        labelSize_ = declare_parameter("label_size", 90.0);
        ellipseSpacing_ = declare_parameter("ellipse_spacing", 120.0);
        ellipseLineWidth_ = declare_parameter("ellipse_line_width", 4.0);
        gpsPointSize_ = declare_parameter("gps_point_size", 6.0);

        // Best effort with a deep queue: compatible with both reliable and
        // best-effort publishers, and absorbs bursts when the bag plays fast.
        const auto sensorQos = rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort();
        imuSub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/imu_uncompensated", sensorQos,
            [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { onImu(*m); });
        gpsSub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", sensorQos,
            [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) { onGps(*m); });
        wheelSub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/vehicle/twist", sensorQos,
            [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) { onWheel(*m); });

        odomPub_ = create_publisher<nav_msgs::msg::Odometry>("/ekf/odom", 10);
        markerPub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/viz/markers", 10);
        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Short names: RViz squeezes spaces in text markers. The README has the legend.
        const char* names[kNumTracks] = {"EKF-A", "EKF-B", "Wheel-odom"};
        const std_msgs::msg::ColorRGBA colors[kNumTracks] = {
            rgba(1.0, 0.5, 0.05), rgba(0.17, 0.75, 0.3), rgba(0.35, 0.63, 1.0)};
        const double labelOffsets[kNumTracks] = {1.2 * labelSize_, 2.4 * labelSize_, -1.2 * labelSize_};
        const char* topics[kNumTracks] = {"/viz/ekf_a/path", "/viz/ekf_b/path", "/viz/wheel_odom/path"};
        for (int i = 0; i < kNumTracks; i++) {
            tracks_[i].name = names[i];
            tracks_[i].color = colors[i];
            tracks_[i].labelOffset = labelOffsets[i];
            tracks_[i].path.header.frame_id = mapFrame_;
            tracks_[i].pub = create_publisher<nav_msgs::msg::Path>(topics[i], 10);
        }
        initPointsMarker(gpsRef_, "gps_reference", rgba(0.85, 0.85, 0.85, 0.5), gpsPointSize_);
        initPointsMarker(rejected_, "gps_rejected", rgba(1.0, 0.1, 0.1), 2.0 * gpsPointSize_);

        RCLCPP_INFO(get_logger(), "Waiting for IMU (bias calibration, %.0f s)", kCalibrationTime);
    }

    ~Ekf2DNode() override {
        RCLCPP_INFO(get_logger(), "EKF-A: %d GPS fixes accepted, %d gated out, %d recoveries",
                    nAcceptedA_, ekfA_.numRejected(), ekfA_.numRecoveries());
        RCLCPP_INFO(get_logger(), "EKF-B: %d GPS fixes accepted, %d gated out, %d recoveries",
                    nAcceptedB_, ekfB_.numRejected(), ekfB_.numRecoveries());
    }

private:
    enum class Mode { Calibrating, WaitingForGps, Running };
    enum { kA = 0, kB = 1, kOdom = 2, kNumTracks = 3 };

    // ---- IMU: calibrate biases, then predict all three estimators ----
    void onImu(const sensor_msgs::msg::Imu& msg) {
        const double t = toSec(msg.header.stamp);
        const double ax = msg.linear_acceleration.x;
        const double omega = -msg.angular_velocity.z;   // mounted z-down: flip sign

        if (mode_ == Mode::Calibrating) {
            if (nCalib_ == 0) {
                firstImuT_ = t;
            }
            if (t - firstImuT_ < kCalibrationTime) {
                sumA_ += ax;
                sumOmega_ += omega;
                nCalib_++;
                return;
            }
            biasA_ = sumA_ / nCalib_;
            biasOmega_ = sumOmega_ / nCalib_;
            mode_ = Mode::WaitingForGps;
            RCLCPP_INFO(get_logger(), "IMU bias from %d samples: accel %.4f m/s^2, gyro %.4f rad/s. "
                        "Waiting for the car to move %.0f m...", nCalib_, biasA_, biasOmega_,
                        kStartDistance);
        }

        if (mode_ == Mode::Running) {
            const double dt = std::min(t - lastImuT_, kMaxDt);
            if (dt > 0.0) {
                const double a = ax - biasA_;
                const double w = omega - biasOmega_;
                ekfA_.predict(a, w, dt);
                ekfB_.predict(a, w, dt);
                // Wheel odometry: drive along the heading at the wheel speed, turn with the gyro.
                odomX_ += latestWheelV_ * std::cos(odomTheta_) * dt;
                odomY_ += latestWheelV_ * std::sin(odomTheta_) * dt;
                odomTheta_ += w * dt;
            }
            if (++imuCount_ % 4 == 0) {   // 200 Hz IMU -> 50 Hz output
                publishOutputs(msg.header.stamp);
            }
        }
        lastImuT_ = t;
    }

    // ---- GPS: clean, convert, then start the estimators or update the EKFs ----
    void onGps(const sensor_msgs::msg::NavSatFix& msg) {
        if (std::isnan(msg.latitude) || std::isnan(msg.longitude)) {
            return;
        }
        const double t = toSec(msg.header.stamp);
        const Utm utm = latLonToUtm(msg.latitude, msg.longitude);

        // Teleport check against the previous fix (as offline: even if that one was a teleport).
        const bool teleport = haveLastFix_ &&
            std::hypot(utm.easting - lastE_, utm.northing - lastN_) /
                std::max(t - lastFixT_, 1e-3) > kMaxGpsSpeed;
        haveLastFix_ = true;
        lastE_ = utm.easting;
        lastN_ = utm.northing;
        lastFixT_ = t;
        if (teleport) {
            return;
        }

        if (!haveOrigin_) {
            haveOrigin_ = true;
            originE_ = utm.easting;
            originN_ = utm.northing;
            originT_ = t;
            RCLCPP_INFO(get_logger(), "First GPS fix: UTM zone %d, local origin set", utm.zone);
        }
        const double east = utm.easting - originE_;
        const double north = utm.northing - originN_;
        addPoint(gpsRef_, east, north);

        if (mode_ == Mode::WaitingForGps && std::hypot(east, north) > kStartDistance) {
            const double theta0 = std::atan2(north, east);
            const double v0 = haveWheel_ ? latestWheelV_
                                         : std::hypot(east, north) / std::max(t - originT_, 1e-3);
            ekfA_.setInitialState(east, north, theta0, v0);
            ekfB_.setInitialState(east, north, theta0, v0);
            odomX_ = east;
            odomY_ = north;
            odomTheta_ = theta0;
            mode_ = Mode::Running;
            RCLCPP_INFO(get_logger(), "Started at (%.1f, %.1f) m, heading %.1f deg, speed %.2f m/s",
                        east, north, theta0 * 180.0 / M_PI, v0);
            return;
        }
        if (mode_ != Mode::Running) {
            return;
        }

        if (ekfA_.update(east, north)) {
            nAcceptedA_++;
        }
        if (ekfB_.update(east, north)) {
            nAcceptedB_++;
        } else {
            addPoint(rejected_, east, north);
        }
    }

    // ---- Wheel speed: EKF-B update, and the speed the wheel odometry drives with ----
    void onWheel(const geometry_msgs::msg::TwistStamped& msg) {
        latestWheelV_ = msg.twist.linear.x;
        haveWheel_ = true;
        if (mode_ == Mode::Running) {
            ekfB_.updateSpeed(latestWheelV_);
        }
    }

    // ---- Output ----
    void publishOutputs(const Time& stamp) {
        publishMainEstimate(stamp);

        const double t = toSec(stamp);
        visualization_msgs::msg::MarkerArray markers;

        if (t - lastPathT_ >= kPathPeriod) {
            lastPathT_ = t;
            const double poses[kNumTracks][3] = {
                {ekfA_.state()(0), ekfA_.state()(1), ekfA_.state()(2)},
                {ekfB_.state()(0), ekfB_.state()(1), ekfB_.state()(2)},
                {odomX_, odomY_, odomTheta_},
            };
            for (int i = 0; i < kNumTracks; i++) {
                appendAndPublish(tracks_[i], stamp, poses[i][0], poses[i][1], poses[i][2]);
                markers.markers.push_back(makeLabel(i, stamp, poses[i][0], poses[i][1]));
            }
            // Current ellipse: bright, always replaced in place.
            markers.markers.push_back(makeEllipse(stamp, "covariance_current", 0,
                                                  rgba(1.0, 0.85, 0.2, 1.0), 2.0 * ellipseLineWidth_));
            // Trail: one faint outline per `ellipse_spacing` meters, so a parked car
            // doesn't stack hundreds of them in one place.
            const double bx = ekfB_.state()(0), by = ekfB_.state()(1);
            if (std::hypot(bx - lastEllipseX_, by - lastEllipseY_) >= ellipseSpacing_) {
                lastEllipseX_ = bx;
                lastEllipseY_ = by;
                markers.markers.push_back(makeEllipse(stamp, "covariance_trail",
                                                      ellipseCount_++ % kMaxEllipses,
                                                      rgba(1.0, 0.8, 0.15, 0.6), ellipseLineWidth_));
            }
        }
        if (t - lastGpsRefT_ >= kGpsRefPeriod) {
            lastGpsRefT_ = t;
            gpsRef_.header.stamp = stamp;
            rejected_.header.stamp = stamp;
            markers.markers.push_back(gpsRef_);
            if (!rejected_.points.empty()) {
                markers.markers.push_back(rejected_);
            }
        }
        if (!markers.markers.empty()) {
            markerPub_->publish(markers);
        }
    }

    // /ekf/odom (with covariance) and TF map -> base_link, from EKF-B.
    void publishMainEstimate(const Time& stamp) {
        const Vec5& x = ekfB_.state();
        const Mat5& P = ekfB_.covariance();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = mapFrame_;
        odom.child_frame_id = baseFrame_;
        odom.pose.pose.position.x = x(0);
        odom.pose.pose.position.y = x(1);
        odom.pose.pose.orientation = yawToQuaternion(x(2));
        // 6x6 row-major over (x, y, z, roll, pitch, yaw). z, roll and pitch are
        // not estimated: give them a large variance.
        auto& c = odom.pose.covariance;
        c[0] = P(0, 0);
        c[1] = P(0, 1);
        c[5] = P(0, 2);
        c[6] = P(1, 0);
        c[7] = P(1, 1);
        c[11] = P(1, 2);
        c[30] = P(2, 0);
        c[31] = P(2, 1);
        c[35] = P(2, 2);
        c[14] = c[21] = c[28] = 1e6;
        odom.twist.twist.linear.x = x(3);
        odom.twist.covariance[0] = P(3, 3);
        odomPub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = baseFrame_;
        tf.transform.translation.x = x(0);
        tf.transform.translation.y = x(1);
        tf.transform.rotation = odom.pose.pose.orientation;
        tfBroadcaster_->sendTransform(tf);
    }

    void appendAndPublish(Track& track, const Time& stamp, double east, double north, double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = mapFrame_;
        pose.pose.position.x = east;
        pose.pose.position.y = north;
        pose.pose.orientation = yawToQuaternion(yaw);
        track.path.poses.push_back(pose);
        track.path.header.stamp = stamp;
        track.pub->publish(track.path);
    }

    Marker makeLabel(int i, const Time& stamp, double east, double north) const {
        Marker m;
        m.header.frame_id = mapFrame_;
        m.header.stamp = stamp;
        m.ns = "labels";
        m.id = i;
        m.type = Marker::TEXT_VIEW_FACING;
        m.action = Marker::ADD;
        m.pose.position.x = east;
        m.pose.position.y = north + tracks_[i].labelOffset;
        m.pose.position.z = 5.0;
        m.pose.orientation.w = 1.0;
        m.scale.z = labelSize_;   // text height (m)
        m.color = tracks_[i].color;
        m.text = tracks_[i].name;
        return m;
    }

    // Outline of the 2-sigma ellipse of EKF-B's position covariance.
    Marker makeEllipse(const Time& stamp, const std::string& ns, int id,
                       const std_msgs::msg::ColorRGBA& color, double lineWidth) const {
        const Eigen::Matrix2d Pxy = ekfB_.covariance().topLeftCorner<2, 2>();
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(Pxy);
        const Eigen::Vector2d lambda = eig.eigenvalues().cwiseMax(0.0);   // ascending
        const Eigen::Vector2d major = eig.eigenvectors().col(1);
        const Eigen::Vector2d minor = eig.eigenvectors().col(0);
        const double a = 2.0 * std::sqrt(lambda(1));   // 2-sigma semi-axes (m)
        const double b = 2.0 * std::sqrt(lambda(0));

        Marker m;
        m.header.frame_id = mapFrame_;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;   // the trail reuses ids, so the oldest ellipse is replaced
        m.type = Marker::LINE_STRIP;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = lineWidth;
        m.color = color;
        const Eigen::Vector2d center(ekfB_.state()(0), ekfB_.state()(1));
        for (int k = 0; k <= kEllipsePoints; k++) {
            const double phi = 2.0 * M_PI * k / kEllipsePoints;
            const Eigen::Vector2d p = center + a * std::cos(phi) * major + b * std::sin(phi) * minor;
            geometry_msgs::msg::Point pt;
            pt.x = p.x();
            pt.y = p.y();
            pt.z = -0.5;   // just below the paths
            m.points.push_back(pt);
        }
        return m;
    }

    void initPointsMarker(Marker& m, const std::string& ns, const std_msgs::msg::ColorRGBA& color,
                          double size) {
        m.header.frame_id = mapFrame_;
        m.ns = ns;
        m.id = 0;
        m.type = Marker::POINTS;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = size;   // point size (m)
        m.pose.position.z = -2.0;       // under the paths, so agreeing tracks stay visible
        m.color = color;
    }

    static void addPoint(Marker& m, double east, double north) {
        geometry_msgs::msg::Point p;
        p.x = east;
        p.y = north;
        m.points.push_back(p);
    }

    // Parameters
    std::string mapFrame_;
    std::string baseFrame_;
    double labelSize_, ellipseSpacing_, ellipseLineWidth_, gpsPointSize_;

    // Estimators and their startup state
    Ekf2D ekfA_;   // IMU + GPS
    Ekf2D ekfB_;   // IMU + wheel + GPS
    double odomX_ = 0.0, odomY_ = 0.0, odomTheta_ = 0.0;
    Mode mode_ = Mode::Calibrating;
    double firstImuT_ = 0.0, lastImuT_ = 0.0;
    double sumA_ = 0.0, sumOmega_ = 0.0, biasA_ = 0.0, biasOmega_ = 0.0;
    int nCalib_ = 0;
    long imuCount_ = 0;

    bool haveOrigin_ = false, haveLastFix_ = false, haveWheel_ = false;
    double originE_ = 0.0, originN_ = 0.0, originT_ = 0.0;
    double lastE_ = 0.0, lastN_ = 0.0, lastFixT_ = 0.0;
    double latestWheelV_ = 0.0;
    int nAcceptedA_ = 0, nAcceptedB_ = 0;

    // Output
    Track tracks_[kNumTracks];
    Marker gpsRef_, rejected_;
    int ellipseCount_ = 0;
    double lastEllipseX_ = 1e9, lastEllipseY_ = 1e9;
    double lastPathT_ = 0.0, lastGpsRefT_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheelSub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ekf2DNode>());
    rclcpp::shutdown();
    return 0;
}
