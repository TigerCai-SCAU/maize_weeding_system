#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

class GnssGateNode : public rclcpp::Node
{
public:
  GnssGateNode() : Node("gnss_gate")
  {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/gnss/odom");
    input_fix_topic_ = declare_parameter<std::string>("input_fix_topic", "/gnss/fix");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/gnss/odom_gated");

    require_fix_ = declare_parameter<bool>("require_fix", true);
    require_covariance_ = declare_parameter<bool>("require_covariance", true);
    max_fix_age_sec_ = declare_parameter<double>("max_fix_age_sec", 1.0);
    max_horizontal_stddev_ = declare_parameter<double>("max_horizontal_stddev", 0.3);
    max_vertical_stddev_ = declare_parameter<double>("max_vertical_stddev", 1.0);
    max_xy_step_ = declare_parameter<double>("max_xy_step", 0.5);
    max_z_step_ = declare_parameter<double>("max_z_step", 1.0);
    output_min_interval_sec_ = declare_parameter<double>("output_min_interval_sec", 0.05);
    min_stable_samples_ = declare_parameter<int>("min_stable_samples", 5);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        input_fix_topic_, 10, std::bind(&GnssGateNode::fixCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_, 10, std::bind(&GnssGateNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "GNSS gate: %s + %s -> %s",
                input_fix_topic_.c_str(), input_odom_topic_.c_str(), output_odom_topic_.c_str());
  }

private:
  static bool finitePosition(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    latest_fix_ = *msg;
    have_fix_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    if (!finitePosition(*msg) || !fixIsUsable(stamp)) {
      stable_samples_ = 0;
      return;
    }

    if (have_last_ && !stepIsUsable(*msg)) {
      stable_samples_ = 0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "GNSS gate rejected position jump");
      return;
    }

    stable_samples_++;
    if (stable_samples_ < min_stable_samples_) {
      return;
    }

    if (have_publish_stamp_) {
      const double dt = (stamp - last_publish_stamp_).seconds();
      if (dt >= 0.0 && dt < output_min_interval_sec_) {
        return;
      }
    }

    odom_pub_->publish(*msg);
    last_accepted_ = *msg;
    last_publish_stamp_ = stamp;
    have_last_ = true;
    have_publish_stamp_ = true;
  }

  bool fixIsUsable(const rclcpp::Time & odom_stamp)
  {
    if (!have_fix_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "GNSS gate waiting for fix topic");
      return false;
    }

    if (require_fix_ && latest_fix_.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "GNSS gate rejected no-fix status");
      return false;
    }

    const rclcpp::Time fix_stamp(latest_fix_.header.stamp);
    const double fix_age = std::fabs((odom_stamp - fix_stamp).seconds());
    if (fix_age > max_fix_age_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "GNSS gate rejected stale fix: age=%.3fs", fix_age);
      return false;
    }

    if (latest_fix_.position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      if (require_covariance_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "GNSS gate rejected unknown fix covariance");
        return false;
      }
      return true;
    }

    const double horizontal_var =
        std::max(latest_fix_.position_covariance[0], latest_fix_.position_covariance[4]);
    const double vertical_var = latest_fix_.position_covariance[8];
    if (!std::isfinite(horizontal_var) || horizontal_var <= 0.0 ||
        !std::isfinite(vertical_var) || vertical_var <= 0.0) {
      return !require_covariance_;
    }

    const double horizontal_stddev = std::sqrt(horizontal_var);
    const double vertical_stddev = std::sqrt(vertical_var);
    if (horizontal_stddev > max_horizontal_stddev_ || vertical_stddev > max_vertical_stddev_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "GNSS gate rejected covariance: h=%.3fm z=%.3fm",
                           horizontal_stddev, vertical_stddev);
      return false;
    }
    return true;
  }

  bool stepIsUsable(const nav_msgs::msg::Odometry & msg) const
  {
    const auto & p = msg.pose.pose.position;
    const auto & last = last_accepted_.pose.pose.position;
    const double dx = p.x - last.x;
    const double dy = p.y - last.y;
    const double dz = p.z - last.z;
    const double xy_step = std::sqrt(dx * dx + dy * dy);
    return xy_step <= max_xy_step_ && std::fabs(dz) <= max_z_step_;
  }

  std::string input_odom_topic_;
  std::string input_fix_topic_;
  std::string output_odom_topic_;
  bool require_fix_;
  bool require_covariance_;
  double max_fix_age_sec_;
  double max_horizontal_stddev_;
  double max_vertical_stddev_;
  double max_xy_step_;
  double max_z_step_;
  double output_min_interval_sec_;
  int min_stable_samples_;

  sensor_msgs::msg::NavSatFix latest_fix_;
  nav_msgs::msg::Odometry last_accepted_;
  rclcpp::Time last_publish_stamp_;
  bool have_fix_ = false;
  bool have_last_ = false;
  bool have_publish_stamp_ = false;
  int stable_samples_ = 0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssGateNode>());
  rclcpp::shutdown();
  return 0;
}
