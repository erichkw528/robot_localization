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
    this->declare_parameter("min_dist", 0.1);

    this->declare_parameter("debug", false);
    if (this->get_parameter("debug").as_bool())
    {
      auto ret = rcutils_logging_set_logger_level(get_logger().get_name(),
                                                  RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
    }

    this->parse_datum();

    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10, std::bind(&LocalizationHack::topic_callback, this, _1));

    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  LocalizationHack::~LocalizationHack()
  {
    // Perform any necessary cleanup here
  }

  void LocalizationHack::topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
  {
    // convert coordinate to cartesian space
    CartesianPosition cartesian_position;
    convert_gnss_to_local_cartesian(gps_msg, cartesian_position);

    // check if this data can be used to compute steering angle
    // if yes, memorize this
    // if not, do nothing
    if (this->latest_cartesian_used_for_steering_ == nullptr)
    {
      this->latest_cartesian_used_for_steering_ = std::make_shared<CartesianPosition>(cartesian_position);
      return;
    }

    RCLCPP_DEBUG(get_logger(), "---------------------");

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = this->get_parameter("map_frame").as_string();
    transformStamped.child_frame_id = this->get_parameter("base_link_frame").as_string();

    // print latest_cartesian_used_for_steering
    RCLCPP_DEBUG(this->get_logger(), "last: [%.6f, %.6f, %.6f]",
                 this->latest_cartesian_used_for_steering_->x,
                 this->latest_cartesian_used_for_steering_->y,
                 this->latest_cartesian_used_for_steering_->z);
    // print cartesian_pos
    RCLCPP_DEBUG(this->get_logger(), "curr: [%.6f, %.6f, %.6f]",
                 cartesian_position.x,
                 cartesian_position.y,
                 cartesian_position.z);

    // compute position
    transformStamped.transform.translation.x = cartesian_position.x;
    transformStamped.transform.translation.y = cartesian_position.y;
    transformStamped.transform.translation.z = cartesian_position.z;

    if (this->is_steering_angle_computable(cartesian_position))
    {
      // compute steering angle
      double angle = std::atan2(cartesian_position.y - this->latest_cartesian_used_for_steering_->y,
                                cartesian_position.x - this->latest_cartesian_used_for_steering_->x);
      geometry_msgs::msg::Quaternion orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), angle));

      transformStamped.transform.rotation = orientation;
      RCLCPP_DEBUG(this->get_logger(), "angle: %.6f", angle);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Steering is not computable, skipping...");
    }

    // publish tf
    tf_broadcaster_->sendTransform(transformStamped);

    RCLCPP_DEBUG(this->get_logger(), "GNSS: [%.6f, %.6f, %.6f]", gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
    RCLCPP_DEBUG(this->get_logger(), "local_coord: [%.6f, %.6f, %.6f]", cartesian_position.x, cartesian_position.y, cartesian_position.z);
    this->latest_cartesian_used_for_steering_ = std::make_shared<CartesianPosition>(cartesian_position);
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

  bool LocalizationHack::is_steering_angle_computable(const CartesianPosition cartesian_position)
  {
    // check if the distance between the current position and the last position is greater than min_dist
    double dist = std::sqrt(std::pow(cartesian_position.x - this->latest_cartesian_used_for_steering_->x, 2) +
                            std::pow(cartesian_position.y - this->latest_cartesian_used_for_steering_->y, 2) +
                            std::pow(cartesian_position.z - this->latest_cartesian_used_for_steering_->z, 2));
    if (dist < this->get_parameter("min_dist").as_double())
    {
      return false;
    }

    return true;
  }

  void LocalizationHack::convert_gnss_to_local_cartesian(sensor_msgs::msg::NavSatFix::ConstSharedPtr input, CartesianPosition &outputCartesianPosition)
  {
    proj.Forward(input->latitude,
                 input->longitude,
                 input->altitude,
                 outputCartesianPosition.x,
                 outputCartesianPosition.y,
                 outputCartesianPosition.z);
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
