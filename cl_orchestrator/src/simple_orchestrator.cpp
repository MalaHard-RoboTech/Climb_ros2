#include "cl_orchestrator/simple_orchestrator.hpp"

namespace simple_orchestrator {

  SimpleOrchestrator::SimpleOrchestrator(const rclcpp::NodeOptions & options)
  : Node("simple_orchestrator", options)
  {
    using namespace std::placeholders;

    this->state_sub = this->create_subscription<std_msgs::msg::String>(
      "state", 10, std::bind(&SimpleOrchestrator::stateCallback, this, _1));
    this->commandstate_sub = this->create_subscription<std_msgs::msg::String>(
      "command_state", 10, std::bind(&SimpleOrchestrator::commandStateCallback, this, _1));
    this->brakestate_sub = this->create_subscription<std_msgs::msg::String>(
      "brake_state", 10, std::bind(&SimpleOrchestrator::brakeStateCallback, this, _1));
    this->endstop_sub = this->create_subscription<std_msgs::msg::Bool>(
      "endstop", 10, std::bind(&SimpleOrchestrator::endstopCallback, this, _1));
    this->ropelength_sub = this->create_subscription<std_msgs::msg::Int32>(
      "rope_length", 10, std::bind(&SimpleOrchestrator::ropeLengthCallback, this, _1));
  }
  
  void SimpleOrchestrator::stateCallback(const std_msgs::msg::String &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "State: %s", msg.data.c_str());
  }
  void SimpleOrchestrator::commandStateCallback(const std_msgs::msg::String &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Command State: %s", msg.data.c_str());
  }
  void SimpleOrchestrator::brakeStateCallback(const std_msgs::msg::String &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Brake State: %s", msg.data.c_str());
  } 
  void SimpleOrchestrator::endstopCallback(const std_msgs::msg::Bool &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Endstop: %s", msg.data ? "true" : "false");
  }
  void SimpleOrchestrator::ropeLengthCallback(const std_msgs::msg::Int32 &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Rope Length: %d", msg.data);
  }
}