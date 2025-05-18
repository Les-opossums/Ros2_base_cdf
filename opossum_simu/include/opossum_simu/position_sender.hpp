#ifndef POSITION_SENDER_HPP_
#define POSITION_SENDER_HPP_

// #include "geometry_msgs/msg/point.hpp"
#include "opossum_msgs/msg/position_map.hpp"
#include "opossum_msgs/srv/string_req.hpp"

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

class PositionSender : public rclcpp::Node
{
public:
  PositionSender();

private:
  // Parameters
  std::vector<std::string> robot_names_;
  std::string real_position_topic_;
  std::string short_motor_srv_;
  std::string update_position_topic_;
  double update_period_;

  // Current positions: map from robot name to its current position
  // (nullptr if not yet received)
  std::unordered_map<std::string, std::shared_ptr<geometry_msgs::msg::Point>> current_pos_;

  // Publishers: one publisher per robot to publish PositionMap messages
  std::unordered_map<std::string, rclcpp::Publisher<opossum_msgs::msg::PositionMap>::SharedPtr> pub_update_position_;

  // Subscriptions (one per robot)
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr> subscriptions_;

  // Timer for publishing positions periodically
  rclcpp::TimerBase::SharedPtr timer_;

  // Initialization methods
  void init_parameters();
  void init_publishers();
  void init_subscribers();
  void init_clients();

  // Callback for updating a robot's position from a Point message
  void update_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg, const std::string & robot_name);

  // Timer callback: publish the PositionMap message for every robot
  void publish_position_callback();
};

#endif  // POSITION_SENDER_HPP_
