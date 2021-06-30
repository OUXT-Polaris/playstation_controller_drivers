#ifndef PLAYSTATION_CONTROLLER_DRIVERS__DUALSENSE_DRIVER_COMPONENT_HPP_
#define PLAYSTATION_CONTROLLER_DRIVERS__DUALSENSE_DRIVER_COMPONENT_HPP_

#include "playstation_controller_drivers/visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_gamecontroller.h>
#include <unordered_map>

namespace playstation_controller_drivers
{

class DualsenseDriverComponent : public rclcpp::Node
{
public:
  PLAYSTATION_CONTROLLER_DRIVERS_PUBLIC
  explicit DualsenseDriverComponent(const rclcpp::NodeOptions & options);

private:
  void timerCallback();
  void colorCallback(const std_msgs::msg::ColorRGBA::SharedPtr msg);
  void colorNameCallback(const std_msgs::msg::String::SharedPtr msg);
  void handleJoyButtonDown(const SDL_Event & e);
  void handleJoyButtonUp(const SDL_Event & e);
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_string_sub_;
  sensor_msgs::msg::Joy joy_;
  std::unordered_map<uint8_t, bool> buttons_;
  rclcpp::TimerBase::SharedPtr timer_;
  SDL_GameController * controller_{nullptr};
  int device_id_{0};
  void getInput();
  std::thread input_thread_;
};

}  // namespace playstation_controller_drivers

#endif  // PLAYSTATION_CONTROLLER_DRIVERS__DUALSENSE_DRIVER_COMPONENT_HPP_
