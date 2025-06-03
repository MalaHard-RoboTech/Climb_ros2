#include <rclcpp/rclcpp.hpp>
#include <cl_orchestrator/simple_orchestrator.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<simple_orchestrator::SimpleOrchestrator>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}