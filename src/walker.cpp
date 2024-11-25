/**
 * @file walker.cpp
 * @author 
 * Harsh Senjaliya (hsenjali@umd.edu)
 * @brief Implements the Walker class and its associated RobotState classes for state-based robot behavior.
 * @version 0.1
 * @date 2024-11-24
 *
 * @copyright 
 * Copyright (c) 2024
 *
 */
#include "walker/walker.hpp"

#define MAX_FORWARD_SPEED 0.2
#define MAX_ROTATION_SPEED 0.4
#define DISTANCE_THRESHOLD 0.7
#define ROTATION_DISTANCE_THRESHOLD 1.4

// Definition of the Walker class
Walker::Walker() : Node("walker") {
  // Initialize publisher to send velocity commands to the cmd_vel topic
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // Initialize subscription to receive LaserScan messages from the scan topic
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&Walker::scanCallback, this, std::placeholders::_1));
  // Set the initial state of the robot to MovingForward
  state_ = std::make_shared<MovingForward>();
  // Initialize the robot's current and previous states
  current_state_ = "MovingForward";
  previous_state_ = "MovingForward";
  // Set the initial rotation direction to false
  rotating_clockwise_ = false;
  RCLCPP_INFO(this->get_logger(), "Walker node initialized.");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", current_state_.c_str());
}

// LaserScan callback function to process incoming data
void Walker::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Define the range of angles considered as "forward" direction
  const double forward_min_angle = -M_PI / 3;
  const double forward_max_angle = M_PI / 3;

  // Initialize minimum detected distance to a very high value
  float min_distance = std::numeric_limits<float>::infinity();

  // Process each laser scan range within the forward direction
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double angle = msg->angle_min + i * msg->angle_increment;
    if (angle >= forward_min_angle && angle <= forward_max_angle) {
      float range = msg->ranges[i];
      if (std::isfinite(range) && range < min_distance) {
        min_distance = range;
      }
    }
  }
  // Execute the current state's behavior using the detected minimum distance
  state_->execute(*this, min_distance);
}

// Utility function to demangle and retrieve a human-readable type name
std::string Walker::demangle(const char *name) {
  int status;
  char *demangled_name = abi::__cxa_demangle(name, 0, 0, &status);
  if (status == 0) {
    return std::string(demangled_name);
  } else {
    return std::string(name);
  }
}

// Function to transition the robot to a new state
void Walker::changeState(std::shared_ptr<RobotState> new_state) {
  previous_state_ = current_state_;
  current_state_ = demangle(typeid(*new_state).name());
  state_ = new_state;
  RCLCPP_INFO(this->get_logger(), "State changed from %s to %s",
              previous_state_.c_str(), current_state_.c_str());
}

// Implementation of the execute method for the MovingForward state
void MovingForward::execute(Walker &context, float min_distance) {
  // Check if the robot detects an obstacle within the threshold
  if (min_distance < DISTANCE_THRESHOLD) {
    if (context.rotating_clockwise_) {
      // Switch to the RotatingCounterClockwise state
      context.changeState(std::make_shared<RotatingCounterClockwise>());
      context.rotating_clockwise_ = false;
    } else {
      // Switch to the RotatingClockwise state
      context.changeState(std::make_shared<RotatingClockwise>());
      context.rotating_clockwise_ = true;
    }
  } else {
    auto msg = geometry_msgs::msg::Twist();
    // Publish linear velocity to move forward
    msg.linear.x = MAX_FORWARD_SPEED;
    context.publisher_->publish(msg);
  }
}

// Implementation of the execute method for the RotatingClockwise state
void RotatingClockwise::execute(Walker &context, float min_distance) {
  // Check if the robot can resume moving forward
  if (min_distance > ROTATION_DISTANCE_THRESHOLD) {
    context.changeState(std::make_shared<MovingForward>());
  } else {
    // Publish angular velocity to rotate clockwise
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = MAX_ROTATION_SPEED;
    context.publisher_->publish(msg);
  }
}

// Implementation of the execute method for the RotatingCounterClockwise state
void RotatingCounterClockwise::execute(Walker &context, float min_distance) {
  // Check if the robot can resume moving forward
  if (min_distance > ROTATION_DISTANCE_THRESHOLD) {
    context.changeState(std::make_shared<MovingForward>());
  } else {
    // Publish angular velocity to rotate counter-clockwise
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = -MAX_ROTATION_SPEED;
    context.publisher_->publish(msg);
  }
}

// Main function to initialize and run the Walker node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}

