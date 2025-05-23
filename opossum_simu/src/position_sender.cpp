#include "opossum_simu/position_sender.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

PositionSender::PositionSender() : Node("position_sender_node")
{
  init_parameters();
  init_publishers();
  init_subscribers();
  init_clients();
  RCLCPP_INFO(this->get_logger(), "Position sender node initialized.");
}

void PositionSender::init_parameters()
{
  // Declare parameters with default values.
  // You can adjust the defaults as necessary.
  this->declare_parameter<std::vector<std::string>>("robot_names", std::vector<std::string>{});
  this->declare_parameter<std::string>("real_position_topic", "");
  this->declare_parameter<std::string>("short_motor_srv", "");
  this->declare_parameter<double>("update_period", 0.1);
  this->declare_parameter<std::string>("update_position_topic", "");

  // Get parameter values
  this->get_parameter("robot_names", robot_names_);
  this->get_parameter("real_position_topic", real_position_topic_);
  this->get_parameter("short_motor_srv", short_motor_srv_);
  this->get_parameter("update_period", update_period_);
  this->get_parameter("update_position_topic", update_position_topic_);

  // Initialize the current positions to nullptr for each robot.
  for (const auto & name : robot_names_) {
    current_pos_[name] = nullptr;
  }
}

void PositionSender::init_publishers()
{
  // Create a publisher for each robot.
  // The topic name is constructed as: "<robot_name>/<update_position_topic>"
  for (const auto & name : robot_names_) {
    std::string topic_name = name + "/" + update_position_topic_;
    pub_update_position_[name] = this->create_publisher<opossum_msgs::msg::PositionMap>(topic_name, 10);
  }
}

void PositionSender::init_subscribers()
{
  // Create a subscription for each robot.
  // The subscription listens on: "/<robot_name>/<real_position_topic>"
  for (const auto & name : robot_names_) {
    std::string topic_name = "/" + name + "/" + real_position_topic_;
    auto sub = this->create_subscription<geometry_msgs::msg::Point>(
      topic_name,
      10,
      [this, name](const geometry_msgs::msg::Point::SharedPtr msg) {
        this->update_pos_callback(msg, name);
      }
    );
    subscriptions_.push_back(sub);
  }

  // Create a timer to periodically call publish_position_callback.
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(update_period_),
    std::bind(&PositionSender::publish_position_callback, this)
  );
}

void PositionSender::init_clients()
{
  for (const auto &name : robot_names_) {
    auto service_name = name + "/" + short_motor_srv_;

    // Create the client for the current robot.
    auto client = this->create_client<opossum_msgs::srv::StringReq>(service_name);

    // Wait for the service to become available.
    while (!client->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "No position to update currently for %s...", name.c_str());
    }

    // Create and populate the request.
    auto request = std::make_shared<opossum_msgs::srv::StringReq::Request>();
    request->data = "GETODOM,1,30.0";

    // Send the request asynchronously.
    auto future = client->async_send_request(request);

    // Spin until the future is complete (with a timeout if desired).
    // Here we use spin_until_future_complete with the node's base interface.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service %s", service_name.c_str());
      continue;
    }

    // Get the response.
    auto response = future.get()->response;
    // Split the response by comma. We expect the response to be something like:
    // "SOMETHING,1.23,4.56,7.89"
    std::vector<std::string> tokens;
    std::istringstream iss(response);
    std::string token;
    while (std::getline(iss, token, ',')) {
      tokens.push_back(token);
    }
    // Make sure we have at least 4 tokens (token[0] is not used, tokens[1-3] are numbers).
    if (tokens.size() < 4) {
      RCLCPP_ERROR(this->get_logger(), "Response format error for %s: %s", name.c_str(), response.c_str());
      continue;
    }
    try {
      geometry_msgs::msg::Point pt;
      pt.x = std::stof(tokens[1]);
      pt.y = std::stof(tokens[2]);
      pt.z = std::stof(tokens[3]);
      current_pos_[name] = std::make_shared<geometry_msgs::msg::Point>(pt);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error parsing response for %s: %s", name.c_str(), e.what());
    }
  }
}

void PositionSender::update_pos_callback(const geometry_msgs::msg::Point::SharedPtr msg, const std::string & robot_name)
{
  // Update the position for the given robot.
  current_pos_[robot_name] = msg;
}

void PositionSender::publish_position_callback()
{
  // For each robot, if its current position is set, publish a PositionMap message.
  for (const auto & name : robot_names_) {
    if (current_pos_[name] != nullptr) {
      opossum_msgs::msg::PositionMap msg;
      // Set the robot's own position.
      msg.robot = *current_pos_[name];

      // For every other robot, append its position to the ennemis list.
      for (const auto & other_name : robot_names_) {
        if (other_name != name) {
          if (current_pos_[other_name] != nullptr) {
            msg.ennemis.push_back(*current_pos_[other_name]);
          } else {
            // If no position is available, push a default-constructed Point.
            msg.ennemis.push_back(geometry_msgs::msg::Point());
          }
        }
      }
      pub_update_position_[name]->publish(msg);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
