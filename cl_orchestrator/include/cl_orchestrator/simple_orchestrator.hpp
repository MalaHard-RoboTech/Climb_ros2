#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

namespace simple_orchestrator {
    class SimpleOrchestrator : public rclcpp::Node
    {
        public: 
            SimpleOrchestrator(const rclcpp::NodeOptions & options);
        private:
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandstate_sub;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr brakestate_sub;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   endstop_sub;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ropelength_sub;

            void stateCallback(const std_msgs::msg::String &msg) const;
            void commandStateCallback(const std_msgs::msg::String &msg) const;
            void brakeStateCallback(const std_msgs::msg::String &msg) const;
            void endstopCallback(const std_msgs::msg::Bool &msg) const;
            void ropeLengthCallback(const std_msgs::msg::Int32 &msg) const;
    };
}