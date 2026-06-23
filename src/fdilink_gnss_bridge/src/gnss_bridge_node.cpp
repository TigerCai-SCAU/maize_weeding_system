#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

class FdilinkGnssBridge : public rclcpp::Node
{
public:
  FdilinkGnssBridge() : Node("fdilink_gnss_bridge")
  {
    input_fix_topic_ = declare_parameter<std::string>("input_fix_topic", "/gps/fix");
    output_fix_topic_ = declare_parameter<std::string>("output_fix_topic", "/gnss/fix");
    input_imu_topic_ = declare_parameter<std::string>("input_imu_topic", "/imu");
    input_ned_odom_topic_ = declare_parameter<std::string>("input_ned_odom_topic", "/NED_odometry");
    output_ned_odom_topic_ = declare_parameter<std::string>("output_ned_odom_topic", "/ins/odom_ned");
    output_enu_odom_topic_ = declare_parameter<std::string>("output_enu_odom_topic", "/gnss/odom");

    navsat_frame_id_ = declare_parameter<std::string>("navsat_frame_id", "navsat_link");
    ned_odom_frame_id_ = declare_parameter<std::string>("ned_odom_frame_id", "ned");
    enu_odom_frame_id_ = declare_parameter<std::string>("enu_odom_frame_id", "map");
    odom_child_frame_id_ = declare_parameter<std::string>("odom_child_frame_id", "ins_imu");
    max_imu_age_sec_ = declare_parameter<double>("max_imu_age_sec", 0.1);
    max_fix_age_sec_ = declare_parameter<double>("max_fix_age_sec", 1.0);

    fallback_fix_xy_stddev_ = declare_parameter<double>("fallback_fix_xy_stddev", 0.5);
    fallback_fix_z_stddev_ = declare_parameter<double>("fallback_fix_z_stddev", 1.0);
    fallback_odom_xy_stddev_ = declare_parameter<double>("fallback_odom_xy_stddev", 0.5);
    fallback_odom_z_stddev_ = declare_parameter<double>("fallback_odom_z_stddev", 1.0);
    fallback_velocity_stddev_ = declare_parameter<double>("fallback_velocity_stddev", 0.2);
    fallback_orientation_stddev_ = declare_parameter<double>("fallback_orientation_stddev", 0.1);
    use_input_fix_covariance_ = declare_parameter<bool>("use_input_fix_covariance", true);

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(output_fix_topic_, 10);
    ned_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_ned_odom_topic_, 10);
    enu_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_enu_odom_topic_, 10);

    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        input_fix_topic_, 10,
        std::bind(&FdilinkGnssBridge::fixCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        input_imu_topic_, 100,
        std::bind(&FdilinkGnssBridge::imuCallback, this, std::placeholders::_1));
    ned_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_ned_odom_topic_, 10,
        std::bind(&FdilinkGnssBridge::nedOdomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "FDILink bridge: %s + %s + %s -> %s, %s",
                input_ned_odom_topic_.c_str(), input_imu_topic_.c_str(),
                input_fix_topic_.c_str(), output_ned_odom_topic_.c_str(),
                output_enu_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(),
                "Assuming input NED odometry is already lever-arm compensated to the INS IMU origin.");
  }

private:
  static bool finite(double value)
  {
    return std::isfinite(value);
  }

  static bool validVariance(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  static bool validQuaternion(const sensor_msgs::msg::Imu & msg)
  {
    const auto & q = msg.orientation;
    const double norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return std::isfinite(norm2) && norm2 > 1e-6;
  }

  static double stampToSec(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static geometry_msgs::msg::Quaternion normalizeQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    geometry_msgs::msg::Quaternion out = q;
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (std::isfinite(norm) && norm > 1e-6) {
      out.w /= norm;
      out.x /= norm;
      out.y /= norm;
      out.z /= norm;
    } else {
      out.w = 1.0;
      out.x = 0.0;
      out.y = 0.0;
      out.z = 0.0;
    }
    return out;
  }

  static geometry_msgs::msg::Quaternion nedToEnuQuaternion(const geometry_msgs::msg::Quaternion & q_ned)
  {
    const auto q = normalizeQuaternion(q_ned);
    const double w = q.w;
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;

    const double r00 = 1.0 - 2.0 * (y * y + z * z);
    const double r01 = 2.0 * (x * y - z * w);
    const double r02 = 2.0 * (x * z + y * w);
    const double r10 = 2.0 * (x * y + z * w);
    const double r11 = 1.0 - 2.0 * (x * x + z * z);
    const double r12 = 2.0 * (y * z - x * w);
    const double r20 = 2.0 * (x * z - y * w);
    const double r21 = 2.0 * (y * z + x * w);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);

    const double e00 = r11;
    const double e01 = r10;
    const double e02 = -r12;
    const double e10 = r01;
    const double e11 = r00;
    const double e12 = -r02;
    const double e20 = -r21;
    const double e21 = -r20;
    const double e22 = r22;

    geometry_msgs::msg::Quaternion out;
    const double trace = e00 + e11 + e22;
    if (trace > 0.0) {
      const double s = std::sqrt(trace + 1.0) * 2.0;
      out.w = 0.25 * s;
      out.x = (e21 - e12) / s;
      out.y = (e02 - e20) / s;
      out.z = (e10 - e01) / s;
    } else if (e00 > e11 && e00 > e22) {
      const double s = std::sqrt(1.0 + e00 - e11 - e22) * 2.0;
      out.w = (e21 - e12) / s;
      out.x = 0.25 * s;
      out.y = (e01 + e10) / s;
      out.z = (e02 + e20) / s;
    } else if (e11 > e22) {
      const double s = std::sqrt(1.0 + e11 - e00 - e22) * 2.0;
      out.w = (e02 - e20) / s;
      out.x = (e01 + e10) / s;
      out.y = 0.25 * s;
      out.z = (e12 + e21) / s;
    } else {
      const double s = std::sqrt(1.0 + e22 - e00 - e11) * 2.0;
      out.w = (e10 - e01) / s;
      out.x = (e02 + e20) / s;
      out.y = (e12 + e21) / s;
      out.z = 0.25 * s;
    }
    return normalizeQuaternion(out);
  }

  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    latest_fix_ = *msg;
    have_fix_ = true;

    sensor_msgs::msg::NavSatFix out = *msg;
    out.header.frame_id = navsat_frame_id_;
    if (!std::isfinite(out.latitude) || !std::isfinite(out.longitude)) {
      out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    } else if (out.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }
    out.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fillFixCovariance(out);
    fix_pub_->publish(out);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_ = *msg;
    have_imu_ = validQuaternion(*msg);
  }

  void nedOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!finite(msg->pose.pose.position.x) ||
        !finite(msg->pose.pose.position.y) ||
        !finite(msg->pose.pose.position.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping /NED_odometry with non-finite position.");
      return;
    }

    nav_msgs::msg::Odometry ned = *msg;
    ned.header.frame_id = ned_odom_frame_id_;
    ned.child_frame_id = odom_child_frame_id_;
    supplementNedOdom(ned);
    ned_odom_pub_->publish(ned);

    nav_msgs::msg::Odometry enu;
    enu.header = ned.header;
    enu.header.frame_id = enu_odom_frame_id_;
    enu.child_frame_id = odom_child_frame_id_;
    enu.pose.pose.position.x = ned.pose.pose.position.y;
    enu.pose.pose.position.y = ned.pose.pose.position.x;
    enu.pose.pose.position.z = -ned.pose.pose.position.z;
    enu.pose.pose.orientation = nedToEnuQuaternion(ned.pose.pose.orientation);
    enu.twist.twist.linear.x = ned.twist.twist.linear.y;
    enu.twist.twist.linear.y = ned.twist.twist.linear.x;
    enu.twist.twist.linear.z = -ned.twist.twist.linear.z;
    enu.twist.twist.angular = ned.twist.twist.angular;
    fillEnuCovariance(ned, enu);
    enu_odom_pub_->publish(enu);
  }

  void supplementNedOdom(nav_msgs::msg::Odometry & odom)
  {
    fillNedCovarianceFromFix(odom);

    const double odom_time = stampToSec(odom.header.stamp);
    if (have_imu_) {
      const double imu_age = std::fabs(odom_time - stampToSec(latest_imu_.header.stamp));
      if (!std::isfinite(imu_age) || imu_age <= max_imu_age_sec_) {
        odom.pose.pose.orientation = normalizeQuaternion(latest_imu_.orientation);
        odom.twist.twist.angular = latest_imu_.angular_velocity;
        copyOrientationCovariance(latest_imu_, odom);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Latest IMU is %.3f s away from NED odom; keeping fallback orientation.",
                             imu_age);
        setFallbackOrientation(odom);
      }
    } else {
      setFallbackOrientation(odom);
    }
  }

  void fillFixCovariance(sensor_msgs::msg::NavSatFix & fix) const
  {
    const bool input_cov_ok =
        use_input_fix_covariance_ &&
        fix.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
        validVariance(fix.position_covariance[0]) &&
        validVariance(fix.position_covariance[4]) &&
        validVariance(fix.position_covariance[8]);

    if (!input_cov_ok) {
      fix.position_covariance.fill(0.0);
      fix.position_covariance[0] = fallback_fix_xy_stddev_ * fallback_fix_xy_stddev_;
      fix.position_covariance[4] = fallback_fix_xy_stddev_ * fallback_fix_xy_stddev_;
      fix.position_covariance[8] = fallback_fix_z_stddev_ * fallback_fix_z_stddev_;
      fix.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
  }

  void fillNedCovarianceFromFix(nav_msgs::msg::Odometry & odom) const
  {
    odom.pose.covariance.fill(0.0);
    odom.twist.covariance.fill(0.0);

    double north_var = fallback_odom_xy_stddev_ * fallback_odom_xy_stddev_;
    double east_var = fallback_odom_xy_stddev_ * fallback_odom_xy_stddev_;
    double down_var = fallback_odom_z_stddev_ * fallback_odom_z_stddev_;
    const double odom_time = stampToSec(odom.header.stamp);

    if (have_fix_) {
      const double fix_age = std::fabs(odom_time - stampToSec(latest_fix_.header.stamp));
      if ((!std::isfinite(fix_age) || fix_age <= max_fix_age_sec_) &&
          latest_fix_.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        north_var = validVariance(latest_fix_.position_covariance[0])
                        ? latest_fix_.position_covariance[0]
                        : north_var;
        east_var = validVariance(latest_fix_.position_covariance[4])
                       ? latest_fix_.position_covariance[4]
                       : east_var;
        down_var = validVariance(latest_fix_.position_covariance[8])
                       ? latest_fix_.position_covariance[8]
                       : down_var;
      }
    }

    odom.pose.covariance[0] = north_var;
    odom.pose.covariance[7] = east_var;
    odom.pose.covariance[14] = down_var;
    odom.twist.covariance[0] = fallback_velocity_stddev_ * fallback_velocity_stddev_;
    odom.twist.covariance[7] = fallback_velocity_stddev_ * fallback_velocity_stddev_;
    odom.twist.covariance[14] = fallback_velocity_stddev_ * fallback_velocity_stddev_;
  }

  void fillEnuCovariance(const nav_msgs::msg::Odometry & ned, nav_msgs::msg::Odometry & enu) const
  {
    enu.pose.covariance.fill(0.0);
    enu.twist.covariance.fill(0.0);
    enu.pose.covariance[0] = ned.pose.covariance[7];
    enu.pose.covariance[7] = ned.pose.covariance[0];
    enu.pose.covariance[14] = ned.pose.covariance[14];
    enu.pose.covariance[21] = ned.pose.covariance[21];
    enu.pose.covariance[28] = ned.pose.covariance[28];
    enu.pose.covariance[35] = ned.pose.covariance[35];
    enu.twist.covariance[0] = ned.twist.covariance[7];
    enu.twist.covariance[7] = ned.twist.covariance[0];
    enu.twist.covariance[14] = ned.twist.covariance[14];
    enu.twist.covariance[21] = ned.twist.covariance[21];
    enu.twist.covariance[28] = ned.twist.covariance[28];
    enu.twist.covariance[35] = ned.twist.covariance[35];
  }

  void copyOrientationCovariance(const sensor_msgs::msg::Imu & imu, nav_msgs::msg::Odometry & odom) const
  {
    odom.pose.covariance[21] = validVariance(imu.orientation_covariance[0])
                                   ? imu.orientation_covariance[0]
                                   : fallback_orientation_stddev_ * fallback_orientation_stddev_;
    odom.pose.covariance[28] = validVariance(imu.orientation_covariance[4])
                                   ? imu.orientation_covariance[4]
                                   : fallback_orientation_stddev_ * fallback_orientation_stddev_;
    odom.pose.covariance[35] = validVariance(imu.orientation_covariance[8])
                                   ? imu.orientation_covariance[8]
                                   : fallback_orientation_stddev_ * fallback_orientation_stddev_;
  }

  void setFallbackOrientation(nav_msgs::msg::Odometry & odom) const
  {
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.covariance[21] = 999999.0;
    odom.pose.covariance[28] = 999999.0;
    odom.pose.covariance[35] = 999999.0;
  }

  std::string input_fix_topic_;
  std::string output_fix_topic_;
  std::string input_imu_topic_;
  std::string input_ned_odom_topic_;
  std::string output_ned_odom_topic_;
  std::string output_enu_odom_topic_;
  std::string navsat_frame_id_;
  std::string ned_odom_frame_id_;
  std::string enu_odom_frame_id_;
  std::string odom_child_frame_id_;
  double max_imu_age_sec_;
  double max_fix_age_sec_;
  double fallback_fix_xy_stddev_;
  double fallback_fix_z_stddev_;
  double fallback_odom_xy_stddev_;
  double fallback_odom_z_stddev_;
  double fallback_velocity_stddev_;
  double fallback_orientation_stddev_;
  bool use_input_fix_covariance_;

  bool have_fix_ = false;
  bool have_imu_ = false;
  sensor_msgs::msg::NavSatFix latest_fix_;
  sensor_msgs::msg::Imu latest_imu_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ned_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr enu_odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ned_odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FdilinkGnssBridge>());
  rclcpp::shutdown();
  return 0;
}
