/*
 * params_helper.h — ROS2 version
 * Adapted from vikit_ros for ROS2 Humble.
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace vk {

// Global node pointer set once during initialization
inline rclcpp::Node::SharedPtr& getParamNode() {
  static rclcpp::Node::SharedPtr node = nullptr;
  return node;
}

inline void setParamNode(rclcpp::Node::SharedPtr node) {
  getParamNode() = node;
}

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
  auto node = getParamNode();
  if (!node) {
    RCLCPP_WARN(rclcpp::get_logger("vikit"), "No node set for param lookup: %s", name.c_str());
    return defaultValue;
  }

  // Convert ROS1 style "/ns/param" to ROS2 style "param"
  std::string param_name = name;
  // Strip leading namespace (e.g., "laserMapping/cam_fx" -> "cam_fx")
  auto pos = param_name.rfind('/');
  if (pos != std::string::npos) {
    param_name = param_name.substr(pos + 1);
  }

  try {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, rclcpp::ParameterValue(defaultValue));
    }
    T v;
    node->get_parameter(param_name, v);
    RCLCPP_INFO(node->get_logger(), "Found parameter: %s", param_name.c_str());
    return v;
  } catch (...) {
    RCLCPP_WARN(node->get_logger(), "Cannot find parameter: %s, using default", param_name.c_str());
    return defaultValue;
  }
}

template<typename T>
T getParam(const std::string& name)
{
  auto node = getParamNode();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("vikit"), "No node set for param lookup: %s", name.c_str());
    return T();
  }

  std::string param_name = name;
  auto pos = param_name.rfind('/');
  if (pos != std::string::npos) {
    param_name = param_name.substr(pos + 1);
  }

  try {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, rclcpp::ParameterValue(T()));
    }
    T v;
    node->get_parameter(param_name, v);
    RCLCPP_INFO(node->get_logger(), "Found parameter: %s", param_name.c_str());
    return v;
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Cannot find parameter: %s", param_name.c_str());
    return T();
  }
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
