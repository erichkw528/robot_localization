#ifndef ROAR_ODOM_PUBLISHER_
#define ROAR_ODOM_PUBLISHER_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace roar
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::NavSatFix>
    ApproximateSyncPolicy;
class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher();
  ~OdomPublisher();

protected:
  void topic_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
                      const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg);

  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;

  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> temp_sync_;
};

}  // namespace roar

#endif  // ROAR_ODOM_PUBLISHER_