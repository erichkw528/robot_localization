#include "roar_robot_localization/localization_hack.hpp"
#include <rmw/qos_profiles.h>
#include "rmw/types.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sstream>

namespace roar
{

  LocalizationHack::LocalizationHack() : Node("localization_hack")
  {
    // variables
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("base_link_frame", "base_link");
    this->declare_parameter("buffer_size", 5);
    this->declare_parameter("rate_millis", 50);
    this->declare_parameter("datum", "0.0,0.0,0.0");

    this->declare_parameter("debug", false);
    if (this->get_parameter("debug").as_bool())
    {
      auto ret = rcutils_logging_set_logger_level(get_logger().get_name(),
                                                  RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
    }

    this->parse_datum();

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
    rclcpp::QoS profile(10); // Set the buffer size to 10
    profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    profile.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);
    // initialize publisher
    odom_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/output/odom", profile); // TODO: @sebastian change to best effort QoS
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

  void LocalizationHack::topic_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg,
                                        const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gps_msg,
                                        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg)
  {
    /**
     * 1. store data in a variable length queue, overwrite previous data if exceed length n
     */
    BufferData buffer_data{imu_msg, gps_msg, pose_msg};
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
    this->updateLatestTransform();
    this->publishLatestTransform();
  }

  void LocalizationHack::updateLatestTransform()
  {
    const BufferData &newData = buffer.back();
    const BufferData &oldestData = buffer.front();

    // latest cartesian coord with respect to datum
    CartesianPosition latest_cartesian_position;
    convert_gnss_to_local_cartesian(newData.gps_msg, latest_cartesian_position);

    CartesianPosition oldest_cartesian_position;
    convert_gnss_to_local_cartesian(oldestData.gps_msg, oldest_cartesian_position);

    // construct the Transform message
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = newData.gps_msg->header.stamp;
    transformStamped.header.frame_id = this->get_parameter("map_frame").as_string();
    transformStamped.child_frame_id = this->get_parameter("base_link_frame").as_string();

    // compute position
    transformStamped.transform.translation.x = latest_cartesian_position.x;
    transformStamped.transform.translation.y = latest_cartesian_position.y;
    transformStamped.transform.translation.z = latest_cartesian_position.z;

    // compute orientation from latest and oldest cartesian position
    // if the difference bewteen old y and new y is too small, angle = 0
    // else, compute the angle
    double angle = 0;

    if (std::abs(latest_cartesian_position.y - oldest_cartesian_position.y) < 0.0001 || std::abs(latest_cartesian_position.x - oldest_cartesian_position.x) < 0.0001)
    {
      RCLCPP_DEBUG(this->get_logger(), "Angle is 0");
    }
    else
    {
      angle = std::atan2(latest_cartesian_position.y - oldest_cartesian_position.y,
                         latest_cartesian_position.x - oldest_cartesian_position.x);
    }

    geometry_msgs::msg::Quaternion orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), angle));
    transformStamped.transform.rotation = orientation;
    // update latest transform
    latest_transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>(transformStamped);

    // print latest transform
    RCLCPP_DEBUG(this->get_logger(), "Latest transform: [%f, %f, %f, %f, %f, %f, %f] | angle: [%f]",
                 latest_transform_->transform.translation.x,
                 latest_transform_->transform.translation.y,
                 latest_transform_->transform.translation.z,
                 latest_transform_->transform.rotation.x,
                 latest_transform_->transform.rotation.y,
                 latest_transform_->transform.rotation.z,
                 latest_transform_->transform.rotation.w, angle);
  }

  void LocalizationHack::publishLatestTransform()
  {
    if (this->latest_transform_ == nullptr)
    {
      return;
    }

    this->tf_broadcaster_->sendTransform(*this->latest_transform_);
  }

  void LocalizationHack::parse_datum()
  {
    std::string datum_str = this->get_parameter("datum").as_string();

    // Split the string using ',' as the delimiter
    std::stringstream ss(datum_str);
    std::string token;
    std::vector<float> values;
    while (std::getline(ss, token, ','))
    {
      try
      {
        float value = std::stof(token);
        values.push_back(value);
      }
      catch (const std::exception &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Error parsing parameter: %s", ex.what());
      }
    }

    // Ensure we have exactly three values
    if (values.size() != 3)
    {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter format for 'datum'. Expected 3 values separated by commas.");
    }
    else
    {
      map_origin_.latitude = values[0];
      map_origin_.longitude = values[1];
      map_origin_.altitude = values[2];
      const GeographicLib::Geocentric &earth = GeographicLib::Geocentric::WGS84();
      proj = GeographicLib::LocalCartesian(map_origin_.latitude, map_origin_.longitude, map_origin_.altitude, earth);
      RCLCPP_INFO(get_logger(), "Datum - Latitude: %.6f, Longitude: %.6f, Altitude: %.6f",
                  map_origin_.latitude, map_origin_.longitude, map_origin_.altitude);
    }
  }
} // namespace roar

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<roar::LocalizationHack>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
