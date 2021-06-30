#include <rclcpp/rclcpp.hpp>
#include <playstation_controller_drivers/dualsense_driver_component.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<playstation_controller_drivers::DualsenseDriverComponent>(
    options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
