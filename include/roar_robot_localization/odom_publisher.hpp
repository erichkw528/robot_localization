#ifndef ROAR_ODOM_PUBLISHER_
#define ROAR_ODOM_PUBLISHER_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <iomanip>  // Include the <iomanip> header for setprecision
#include <sstream>

namespace roar
{

struct BufferData
{
  sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg;
};

struct GpsPosition
{
  double latitude;
  double longitude;
  double altitude;
};

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::NavSatFix,
                                                        geometry_msgs::msg::PoseStamped>
    ApproximateSyncPolicy;
class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher();
  ~OdomPublisher();

protected:
  // subscribers
  void topic_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
                      const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                      const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg);
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> temp_sync_;

  // publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  void publish_transform(const nav_msgs::msg::Odometry::SharedPtr odom);
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  // data
  std::vector<BufferData> buffer;
  int bufferSize = 10;
  GpsPosition map_origin_;

  /**
   * Convert data using GeographicLib
   * using imu + pose to estimate linear speed
   * using imu to get orientation and angular
   */
  void p_calcOdom(const std::vector<BufferData>* buffer, nav_msgs::msg::Odometry::SharedPtr odom);

  std::string to_string(const nav_msgs::msg::Odometry& odom)
  {
    std::ostringstream oss;
    oss << "  Stamp: " << odom.header.stamp.sec << "." << std::setfill('0') << std::setw(9) << odom.header.stamp.nanosec
        << std::endl;
    oss << "  Position:";
    oss << "    x: " << odom.pose.pose.position.x;
    oss << "    y: " << odom.pose.pose.position.y;
    oss << "    z: " << odom.pose.pose.position.z;
    oss << "" << std::endl;
    oss << "  Orientation:";
    oss << "    x: " << odom.pose.pose.orientation.x;
    oss << "    y: " << odom.pose.pose.orientation.y;
    oss << "    z: " << odom.pose.pose.orientation.z;
    oss << "    w: " << odom.pose.pose.orientation.w << std::endl;
    oss << "Twist  Linear:";
    oss << "    x: " << odom.twist.twist.linear.x;
    oss << "    y: " << odom.twist.twist.linear.y;
    oss << "    z: " << odom.twist.twist.linear.z << std::endl;
    oss << "Twist  Angular:";
    oss << "    x: " << odom.twist.twist.angular.x;
    oss << "    y: " << odom.twist.twist.angular.y;
    oss << "    z: " << odom.twist.twist.angular.z << std::endl;
    // oss << "" << std::endl;
    return oss.str();
  }
};

}  // namespace roar

#endif  // ROAR_ODOM_PUBLISHER_