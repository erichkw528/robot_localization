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
#include <iomanip> // Include the <iomanip> header for setprecision
#include <sstream>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
namespace roar
{

  struct BufferData
  {
    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg;
  };

  struct CartesianPosition
  {
    double x;
    double y;
    double z;
  };
  struct GeodeticPosition
  {
    double latitude;
    double longitude;
    double altitude;
  };

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::NavSatFix,
                                                          geometry_msgs::msg::PoseStamped>
      ApproximateSyncPolicy;
  class LocalizationHack : public rclcpp::Node
  {
  public:
    LocalizationHack();
    ~LocalizationHack();

  protected:
    // subscribers
    void topic_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
                        const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gps_msg,
                        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg);
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> temp_sync_;

    // publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    void updateLatestTransform();
    void publishLatestTransform();
    void parse_datum();

    std::unique_ptr<tf2_ros::TransformBroadcaster>
        tf_broadcaster_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    // data
    std::vector<BufferData> buffer;
    int bufferSize = 10;
    GeographicLib::LocalCartesian proj;
    GeodeticPosition map_origin_;

    std::shared_ptr<geometry_msgs::msg::TransformStamped> latest_transform_;

    void convert_gnss_to_local_cartesian(sensor_msgs::msg::NavSatFix::ConstSharedPtr input, CartesianPosition &outputCartesianPosition)
    {

      proj.Forward(input->latitude,
                   input->longitude,
                   input->altitude,
                   outputCartesianPosition.x,
                   outputCartesianPosition.y,
                   outputCartesianPosition.z);
    }

    std::string to_string(const nav_msgs::msg::Odometry &odom)
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

} // namespace roar

#endif // ROAR_ODOM_PUBLISHER_