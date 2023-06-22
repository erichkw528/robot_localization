#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"

class PoseConverterNode : public rclcpp::Node
{
public:
  PoseConverterNode() : Node("pose_converter_node")
  {
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "input_pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) { convertPoseToPoseWithCovariance(pose); });
    pose_with_covariance_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("output_pose_with_covariance", 10);
  }

private:
  void convertPoseToPoseWithCovariance(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
    pose_with_covariance.header = pose->header;
    pose_with_covariance.pose.pose = pose->pose;
    // Set covariance values to 1
    pose_with_covariance.pose.covariance.fill(0.0);
    for (size_t i = 0; i < 6; ++i)
    {
      pose_with_covariance.pose.covariance[i * 7] = 1.0;
    }

    pose_with_covariance_publisher_->publish(pose_with_covariance);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseConverterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}