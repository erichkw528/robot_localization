#ifndef LOCALIZATION_HACK_
#define LOCALIZATION_HACK_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <iomanip> // Include the <iomanip> header for setprecision
#include <sstream>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

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
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> temp_sync_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);
    // publisher
    void parse_datum();

    std::unique_ptr<tf2_ros::TransformBroadcaster>
        tf_broadcaster_;

    // data
    GeographicLib::LocalCartesian proj;
    GeodeticPosition map_origin_;

    std::shared_ptr<geometry_msgs::msg::TransformStamped> latest_transform_;
    std::shared_ptr<CartesianPosition> latest_cartesian_used_for_steering_;
    bool is_steering_angle_computable(CartesianPosition cartesian_position)
    {
      if (latest_cartesian_used_for_steering_ == nullptr)
      {
        return false;
      }
      double distance = sqrt(pow(cartesian_position.x - latest_cartesian_used_for_steering_->x, 2) +
                             pow(cartesian_position.y - latest_cartesian_used_for_steering_->y, 2));
      if (distance < 0.01)
      {
        return false;
      }
      return true;
    }

    void convert_gnss_to_local_cartesian(sensor_msgs::msg::NavSatFix::ConstSharedPtr input, CartesianPosition &outputCartesianPosition)
    {

      proj.Forward(input->latitude,
                   input->longitude,
                   input->altitude,
                   outputCartesianPosition.x,
                   outputCartesianPosition.y,
                   outputCartesianPosition.z);
    }
  };

} // namespace roar

#endif // LOCALIZATION_HACK_