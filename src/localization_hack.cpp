#include "roar_robot_localization/localization_hack.hpp"
#include <rmw/qos_profiles.h>
#include "rmw/types.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"

#include <sstream>

namespace roar
{

LocalizationHack::LocalizationHack() : Node("localization_hack")
{
  // variables
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_link_frame", "base_link");
  this->declare_parameter("buffer_size", 5);
  this->declare_parameter("rate_millis", 50);
  this->declare_parameter("debug", false);
  if (this->get_parameter("debug").as_bool())
  {
    auto ret = rcutils_logging_set_logger_level(get_logger().get_name(),
                                                RCUTILS_LOG_SEVERITY_DEBUG);  // enable or disable debug
  }

  map_origin_ = { declare_parameter("map_origin.latitude", 0.0), declare_parameter("map_origin.longitude", 0.0),
                  declare_parameter("map_origin.altitude", 0.0) };

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
  temp_sync_->registerCallback(std::bind(&LocalizationHack::topic_callback, this, std::placeholders::_1,
                                         std::placeholders::_2, std::placeholders::_3));
  // Create a custom QoS profile
  rclcpp::QoS profile(10);  // Set the buffer size to 10
  profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  profile.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);
  // initialize publisher
  odom_publisher_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/output/odom", profile);  // TODO: @sebastian change to best effort QoS
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  timer_ = create_wall_timer(std::chrono::milliseconds(this->get_parameter("rate_millis").as_int()),
                             std::bind(&LocalizationHack::timer_callback, this));

  // initialize buffer
  this->bufferSize = this->get_parameter("buffer_size").as_int();

  RCLCPP_INFO(get_logger(), "OdomPublisher initialized. BufferSize = [%d]", this->bufferSize);
}

LocalizationHack::~LocalizationHack()
{
  // Perform any necessary cleanup here
}

void LocalizationHack::topic_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
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
void LocalizationHack::timer_callback()
{
  if (buffer.size() < bufferSize)
  {
    RCLCPP_DEBUG(this->get_logger(), "Buffer size [%d] < [%d]", buffer.size(), bufferSize);
    return;
  }
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();

  this->p_calcOdom(&this->buffer, odom);
  this->publish_transform(odom);
  this->odom_publisher_->publish(*odom);

  RCLCPP_DEBUG(get_logger(), to_string(*odom));
}

void LocalizationHack::p_calcOdom(const std::vector<BufferData>* buffer, nav_msgs::msg::Odometry::SharedPtr odom)
{
  // Get the latest data from the buffer
  const BufferData& latestData = buffer->back();

  // Fill in the odometry message with the calculated average values
  odom->header.stamp = buffer->back().imu_msg->header.stamp;
  odom->header.frame_id = this->get_parameter("odom_frame").as_string();
  odom->child_frame_id = this->get_parameter("base_link_frame").as_string();

  // Calculate the average values
  double avg_lat = 0.0;
  double avg_long = 0.0;
  double avg_alt = 0.0;

  double avgVx = 0.0;
  double avgVy = 0.0;
  double avgVz = 0.0;

  double angular_velocity_x = 0.0;
  double angular_velocity_y = 0.0;
  double angular_velocity_z = 0.0;

  for (const BufferData& data : *buffer)
  {
    avg_lat += data.gps_msg->latitude;
    avg_long += data.gps_msg->longitude;
    avg_alt += data.gps_msg->altitude;

    angular_velocity_x += data.imu_msg->angular_velocity.x;
    angular_velocity_y += data.imu_msg->angular_velocity.y;
    angular_velocity_z += data.imu_msg->angular_velocity.z;
  }
  // Calculate the average velocity
  if (bufferSize > 1)
  {
    const BufferData& firstData = buffer->front();
    const BufferData& lastData = buffer->back();
    double dt = (lastData.pose_msg->header.stamp.nanosec - firstData.pose_msg->header.stamp.nanosec) /
                1e9;  // Convert to seconds

    avgVx = (lastData.pose_msg->pose.position.x - firstData.pose_msg->pose.position.x) / dt;
    avgVy = (lastData.pose_msg->pose.position.y - firstData.pose_msg->pose.position.y) / dt;
    avgVz = (lastData.pose_msg->pose.position.z - firstData.pose_msg->pose.position.z) / dt;
  }

  size_t bufferSize = buffer->size();
  avg_lat /= bufferSize;
  avg_long /= bufferSize;
  avg_alt /= bufferSize;
  angular_velocity_x /= bufferSize;
  angular_velocity_y /= bufferSize;
  angular_velocity_z /= bufferSize;

  const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
  GeographicLib::LocalCartesian proj(map_origin_.latitude, map_origin_.longitude, map_origin_.altitude, earth);
  proj.Forward(avg_lat, avg_long, avg_alt, odom->pose.pose.position.x, odom->pose.pose.position.y,
               odom->pose.pose.position.z);

  //    proj.Forward(latestData.gps_msg->latitude, latestData.gps_msg->longitude, latestData.gps_msg->altitude,
  //    odom->pose.pose.position.x, odom->pose.pose.position.y,
  //  odom->pose.pose.position.z);

  odom->pose.pose.orientation = latestData.imu_msg->orientation;

  // Fill in the velocity
  odom->twist.twist.linear.x = avgVx;
  odom->twist.twist.linear.y = avgVy;
  odom->twist.twist.linear.z = avgVz;

  odom->twist.twist.angular.x = angular_velocity_x;
  odom->twist.twist.angular.y = angular_velocity_y;
  odom->twist.twist.angular.z = angular_velocity_z;
}

void LocalizationHack::publish_transform(const nav_msgs::msg::Odometry::SharedPtr odom)
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
  auto node = std::make_shared<roar::LocalizationHack>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
