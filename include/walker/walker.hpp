/**
 * @file walker.hpp
 * @author 
 * Harsh Senjaliya (hsenjali@umd.edu)
 * @brief Declares the Walker class and the RobotState class for implementing the walker behavior
 * @version 0.1
 * @date 2024-11-22
 *
 * @copyright 
 * Copyright (c) 2024
 *
 */
#pragma once

#ifndef WALKER_HPP_
#define WALKER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cxxabi.h>

// Forward declaration of Walker class
class Walker;

/**
 * @brief Abstract base class representing the state of the robot.
 * @details RobotState defines an interface for implementing various robot behaviors or states.
 */
class RobotState {
 public:
  /**
   * @brief Executes the behavior associated with the current state of the robot.
   * @param context Reference to the Walker class controlling the robot.
   * @param min_distance Minimum distance detected by the robot's sensors.
   */
  virtual void execute(Walker &context, float min_distance) = 0;

  /// Virtual destructor for proper cleanup of derived classes.
  virtual ~RobotState() = default;
};

/**
 * @brief Represents the robot's "moving forward" behavior.
 * @details Implements logic for the robot's forward movement when no obstacles are nearby.
 */
class MovingForward : public RobotState {
  /**
   * @brief Executes the forward movement behavior.
   * @param context Reference to the Walker class controlling the robot.
   * @param min_distance Minimum distance detected by the robot's sensors.
   */
  void execute(Walker &context, float min_distance) override;
};

/**
 * @brief Represents the robot's "rotating clockwise" behavior.
 * @details Implements logic for rotating the robot in a clockwise direction to avoid obstacles.
 */
class RotatingClockwise : public RobotState {
  /**
   * @brief Executes the clockwise rotation behavior.
   * @param context Reference to the Walker class controlling the robot.
   * @param min_distance Minimum distance detected by the robot's sensors.
   */
  void execute(Walker &context, float min_distance) override;
};

/**
 * @brief Represents the robot's "rotating counter-clockwise" behavior.
 * @details Implements logic for rotating the robot in a counter-clockwise direction to avoid obstacles.
 */
class RotatingCounterClockwise : public RobotState {
  /**
   * @brief Executes the counter-clockwise rotation behavior.
   * @param context Reference to the Walker class controlling the robot.
   * @param min_distance Minimum distance detected by the robot's sensors.
   */
  void execute(Walker &context, float min_distance) override;
};

/**
 * @brief Manages the robot's overall behavior and state transitions.
 * @details Walker class contains the ROS2 node, publishers, subscribers, and manages the robot's state.
 */
class Walker : public rclcpp::Node {
 public:
  /// Constructor to initialize the Walker node and set up ROS2 communication.
  Walker();

  /**
   * @brief Callback function for processing laser scan data.
   * @param msg Shared pointer to the LaserScan message received from the sensor.
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Changes the robot's current state.
   * @param new_state Shared pointer to the new state object to transition to.
   */
  void changeState(std::shared_ptr<RobotState> new_state);

  /**
   * @brief Utility function to retrieve a human-readable class name.
   * @param name Type name obtained from the C++ typeid operator.
   * @return A demangled string representation of the class name.
   */
  std::string demangle(const char *name);

  /// Publisher for sending velocity commands to the robot.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  /// Subscriber for receiving laser scan data from the robot's sensors.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  /// String representing the robot's current state.
  std::string current_state_;

  /// String representing the robot's previous state before the last transition.
  std::string previous_state_;

  /// Shared pointer to the current state object.
  std::shared_ptr<RobotState> state_;

  /// Boolean indicating the robot's last rotation direction (clockwise or not).
  bool rotating_clockwise_;
};

#endif  // WALKER_HPP_

