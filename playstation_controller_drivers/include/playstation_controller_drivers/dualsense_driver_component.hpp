// Copyright (c) 2021 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLAYSTATION_CONTROLLER_DRIVERS__DUALSENSE_DRIVER_COMPONENT_HPP_
#define PLAYSTATION_CONTROLLER_DRIVERS__DUALSENSE_DRIVER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>

#include <SDL2/SDL.h>
#include <SDL2/SDL_gamecontroller.h>
#include <playstation_controller_drivers/visibility_control.h>

#include <unordered_map>
#include <limits>
#include <memory>


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
  void handleJoyAxis(const SDL_Event & e);
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_string_sub_;
  sensor_msgs::msg::Joy joy_;
  std::unordered_map<uint8_t, bool> buttons_;
  std::unordered_map<uint8_t, double> axis_;
  rclcpp::TimerBase::SharedPtr timer_;
  SDL_GameController * controller_{nullptr};
  int device_id_{0};
  void getInput();
  std::thread input_thread_;
  double deadzone_;
  double normalizeUint16Value(int16_t value) const;
};

}  // namespace playstation_controller_drivers

#endif  // PLAYSTATION_CONTROLLER_DRIVERS__DUALSENSE_DRIVER_COMPONENT_HPP_
