#include "roar_robot_localization/odom_publisher.hpp"
#include <rmw/qos_profiles.h>
#include "rmw/types.h"

namespace roar
{

OdomPublisher::OdomPublisher() : Node("odom_publisher_node")
{
  // Initialize your subscribers and synchronizer here
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  imu_sub_.subscribe(this, "/gps/imu", custom_qos_profile);
  // imu_sub_.subscribe(this, "/imu_topic");
  gps_sub_.subscribe(this, "/gps/fix", custom_qos_profile);
  pose_sub_.subscribe(this, "/gps/pose", custom_qos_profile);

  temp_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(20),
                                                                                      imu_sub_, gps_sub_);

  temp_sync_->registerCallback(
      std::bind(&OdomPublisher::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(get_logger(), "OdomPublisher initialized");
}

OdomPublisher::~OdomPublisher()
{
  // Perform any necessary cleanup here
}

void OdomPublisher::topic_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
                                   const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg)
{
  RCLCPP_INFO(get_logger(), "both arrived");
}

}  // namespace roar

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<roar::OdomPublisher>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
