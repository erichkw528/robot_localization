#include "roar_robot_localization/odom_publisher.hpp"
#include <rmw/qos_profiles.h>
#include "rmw/types.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
namespace roar
{

OdomPublisher::OdomPublisher() : Node("odom_publisher_node")
{
  // variables
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_link_frame", "base_link");
  this->declare_parameter("buffer_size", 10);

  // Initialize subscribers and synchronizer here
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  imu_sub_.subscribe(this, "/gps/imu", custom_qos_profile);
  gps_sub_.subscribe(this, "/gps/fix", custom_qos_profile);
  pose_sub_.subscribe(this, "/gps/pose", custom_qos_profile);
  temp_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(ApproximateSyncPolicy(100),
                                                                                      imu_sub_, gps_sub_, pose_sub_);
  temp_sync_->registerCallback(std::bind(&OdomPublisher::topic_callback, this, std::placeholders::_1,
                                         std::placeholders::_2, std::placeholders::_3));

  // initialize publisher
  odom_publisher_ =
      this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);  // TODO: @sebastian change to best effort QoS
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&OdomPublisher::timer_callback, this));

  // initialize buffer
  this->bufferSize = this->get_parameter("buffer_size").as_int();

  RCLCPP_INFO(get_logger(), "OdomPublisher initialized. BufferSize = [%d]", this->bufferSize);
}

OdomPublisher::~OdomPublisher()
{
  // Perform any necessary cleanup here
}

void OdomPublisher::topic_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
                                   const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
                                   const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg)
{
  /**
   * 1. store data in a variable length queue, overwrite previous data if exceed length n
   */
  BufferData buffer_data{ imu_msg, gps_msg, pose_msg };
  buffer.push_back(buffer_data);
  if (buffer.size() > bufferSize)
  {
    buffer.erase(buffer.begin());
  }
}

/**
 * 1. if the buffer is not full full, else, don't do anything (early return)
 * 2. compute the velocity
 * 3. construct the nav_msgs/Odometry object
 * 4. publish transformation from odom -> base_link
 * 5. publish nav_msgs/Odom
 */
void OdomPublisher::timer_callback()
{
  RCLCPP_INFO(get_logger(), "buffer_size: [%d]", buffer.size());
  if (buffer.size() < bufferSize)
  {
    return;
  }
  // TODO: @sebatisan, finish impl here
}

void OdomPublisher::publish_transform(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  geometry_msgs::msg::TransformStamped t;

  // Fill in the necessary fields of the TransformStamped message
  t.header.stamp = this->now();
  t.header.frame_id = this->get_parameter("odom_frame").as_string();
  t.child_frame_id = this->get_parameter("base_link_frame").as_string();
  t.transform.translation.x = odom->pose.pose.position.x;
  t.transform.translation.y = odom->pose.pose.position.y;
  t.transform.translation.z = odom->pose.pose.position.z;
  t.transform.rotation.w = odom->pose.pose.orientation.w;
  t.transform.rotation.x = odom->pose.pose.orientation.x;
  t.transform.rotation.y = odom->pose.pose.orientation.y;
  t.transform.rotation.z = odom->pose.pose.orientation.z;

  // Publish the TransformStamped message
  tf_broadcaster_->sendTransform(t);
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
