#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mrnodetest");

    // Publishers
    auto state_pub = node->create_publisher<std_msgs::msg::String>("state", 10);
    auto command_state_pub = node->create_publisher<std_msgs::msg::String>("command_state", 10);
    auto brake_state_pub = node->create_publisher<std_msgs::msg::String>("brake_state", 10);
    auto endstop_pub = node->create_publisher<std_msgs::msg::Bool>("endstop", 10);
    auto rope_length_pub = node->create_publisher<std_msgs::msg::Int32>("rope_length", 10);

    rclcpp::Rate rate(1);  // 1 Hz

    int counter = 0;
    while (rclcpp::ok() && counter < 5)
    {
        // String messages
        std_msgs::msg::String state_msg;
        state_msg.data = "idle";
        state_pub->publish(state_msg);

        std_msgs::msg::String command_state_msg;
        command_state_msg.data = "start";
        command_state_pub->publish(command_state_msg);

        std_msgs::msg::String brake_state_msg;
        brake_state_msg.data = "engaged";
        brake_state_pub->publish(brake_state_msg);

        // Bool message
        std_msgs::msg::Bool endstop_msg;
        endstop_msg.data = (counter % 2 == 0);  // Alternate true/false
        endstop_pub->publish(endstop_msg);

        // Int message
        std_msgs::msg::Int32 rope_length_msg;
        rope_length_msg.data = 100 + counter * 10;
        rope_length_pub->publish(rope_length_msg);

        RCLCPP_INFO(node->get_logger(), "Published test messages (round %d)", counter + 1);

        rate.sleep();
        counter++;
    }

    rclcpp::shutdown();
    return 0;
}
