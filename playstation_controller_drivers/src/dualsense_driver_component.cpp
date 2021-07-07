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

#include <playstation_controller_drivers/dualsense_driver_component.hpp>
#include <playstation_controller_drivers/util.hpp>
#include <color_names/color_names.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace playstation_controller_drivers
{

DualsenseDriverComponent::DualsenseDriverComponent(const rclcpp::NodeOptions & options)
: Node("dualsense_driver", options)
{
  declare_parameter("deadzone", 0.1);
  get_parameter("deadzone", deadzone_);
  color_sub_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
    "~/color", 1,
    std::bind(&DualsenseDriverComponent::colorCallback, this, std::placeholders::_1));
  color_string_sub_ = this->create_subscription<std_msgs::msg::String>(
    "~/color/name", 1,
    std::bind(&DualsenseDriverComponent::colorNameCallback, this, std::placeholders::_1));
  joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 1);
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(10ms, std::bind(&DualsenseDriverComponent::timerCallback, this));
  input_thread_ = std::thread(&DualsenseDriverComponent::getInput, this);
}

void DualsenseDriverComponent::colorCallback(const std_msgs::msg::ColorRGBA::SharedPtr msg)
{
  const auto color = LEDColor(msg);
  SDL_GameControllerSetLED(controller_, color.r, color.g, color.b);
}

void DualsenseDriverComponent::colorNameCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const auto color = LEDColor(color_names::makeColorMsg(msg->data));
  SDL_GameControllerSetLED(controller_, color.r, color.g, color.b);
}

void DualsenseDriverComponent::handleJoyButtonDown(const SDL_Event & e)
{
  const auto it = buttons_.find(e.jbutton.button);
  if (it == buttons_.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Unregisterd button was pressed.");
    return;
  }
  buttons_[e.jbutton.button] = true;
}

void DualsenseDriverComponent::handleJoyButtonUp(const SDL_Event & e)
{
  const auto it = buttons_.find(e.jbutton.button);
  if (it == buttons_.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Unregisterd button was pressed.");
    return;
  }
  buttons_[e.jbutton.button] = false;
}

void DualsenseDriverComponent::handleJoyAxis(const SDL_Event & e)
{
  const auto it = axis_.find(e.jaxis.axis);
  if (it == axis_.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Unregisterd button was pressed.");
    return;
  }
  axis_[e.jaxis.axis] = normalizeUint16Value(e.jaxis.value);
}

double DualsenseDriverComponent::normalizeUint16Value(int16_t value) const
{
  constexpr int16_t max = std::numeric_limits<int16_t>::max();
  constexpr int16_t min = std::numeric_limits<int16_t>::min();
  double raw_value = (static_cast<double>(value - min) / static_cast<double>(max - min) - 0.5) * -2;
  if (deadzone_ >= std::fabs(raw_value)) {
    raw_value = 0;
  }
  return raw_value;
}

void DualsenseDriverComponent::getInput()
{
  while (rclcpp::ok()) {
    if (controller_ != nullptr) {
      SDL_Event e;
      SDL_WaitEventTimeout(&e, 10);
      if (e.type == SDL_CONTROLLERAXISMOTION) {
        handleJoyAxis(e);
      } else if (e.type == SDL_CONTROLLERBUTTONDOWN) {
        handleJoyButtonDown(e);
      } else if (e.type == SDL_CONTROLLERBUTTONUP) {
        handleJoyButtonUp(e);
      }
    }
  }
}

void DualsenseDriverComponent::timerCallback()
{
  if (controller_ == nullptr) {
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "failed to initialize SDL2");
      return;
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "SDL2 initialized");
      if (!SDL_IsGameController(device_id_)) {
        RCLCPP_ERROR_STREAM(get_logger(), "GameController does not find.");
        throw std::runtime_error("GameController does not find.");
      }
      controller_ = SDL_GameControllerOpen(device_id_);
      if (controller_ == nullptr) {
        RCLCPP_ERROR_STREAM(get_logger(), "GameController is nullptr.");
        throw std::runtime_error("GameController is nullptr.");
      }
      if (SDL_GameControllerGetType(controller_) != SDL_CONTROLLER_TYPE_PS5) {
        RCLCPP_ERROR_STREAM(get_logger(), "GameController is not a dualsense controller.");
        throw std::runtime_error("GameController is not a dualsense controller.");
      }
      buttons_[SDL_CONTROLLER_BUTTON_A] = false;
      buttons_[SDL_CONTROLLER_BUTTON_B] = false;
      buttons_[SDL_CONTROLLER_BUTTON_X] = false;
      buttons_[SDL_CONTROLLER_BUTTON_Y] = false;
      buttons_[SDL_CONTROLLER_BUTTON_BACK] = false;
      buttons_[SDL_CONTROLLER_BUTTON_GUIDE] = false;
      buttons_[SDL_CONTROLLER_BUTTON_START] = false;
      buttons_[SDL_CONTROLLER_BUTTON_DPAD_UP] = false;
      buttons_[SDL_CONTROLLER_BUTTON_DPAD_DOWN] = false;
      buttons_[SDL_CONTROLLER_BUTTON_DPAD_LEFT] = false;
      buttons_[SDL_CONTROLLER_BUTTON_DPAD_RIGHT] = false;
      buttons_[SDL_CONTROLLER_BUTTON_LEFTSTICK] = false;
      buttons_[SDL_CONTROLLER_BUTTON_RIGHTSTICK] = false;
      buttons_[SDL_CONTROLLER_BUTTON_LEFTSHOULDER] = false;
      buttons_[SDL_CONTROLLER_BUTTON_RIGHTSHOULDER] = false;
      axis_[SDL_CONTROLLER_AXIS_LEFTX] = 0;
      axis_[SDL_CONTROLLER_AXIS_LEFTY] = 0;
      axis_[SDL_CONTROLLER_AXIS_RIGHTX] = 0;
      axis_[SDL_CONTROLLER_AXIS_RIGHTY] = 0;
      axis_[SDL_CONTROLLER_AXIS_TRIGGERLEFT] = 0;
      axis_[SDL_CONTROLLER_AXIS_TRIGGERRIGHT] = 0;
    }
  } else {
    sensor_msgs::msg::Joy joy;
    joy.header.frame_id = "joy";
    joy.header.stamp = get_clock()->now();
    joy.buttons = std::vector<int>(buttons_.size());
    joy.axes = std::vector<float>(axis_.size());
    for (const auto & button : buttons_) {
      switch (button.first) {
        case SDL_CONTROLLER_BUTTON_A:
          joy.buttons[1] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_B:
          joy.buttons[3] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_X:
          joy.buttons[2] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_Y:
          joy.buttons[0] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_DPAD_UP:
          joy.buttons[4] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
          joy.buttons[5] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
          joy.buttons[6] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
          joy.buttons[7] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_BACK:
          joy.buttons[8] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_GUIDE:
          joy.buttons[10] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_START:
          joy.buttons[9] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_LEFTSTICK:
          joy.buttons[11] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_RIGHTSTICK:
          joy.buttons[12] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
          joy.buttons[13] = button.second;
          break;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
          joy.buttons[14] = button.second;
          break;
        default:
          break;
      }
    }
    for (const auto & axis : axis_) {
      switch (axis.first) {
        case SDL_CONTROLLER_AXIS_LEFTX:
          joy.axes[0] = axis.second;
          break;
        case SDL_CONTROLLER_AXIS_LEFTY:
          joy.axes[1] = axis.second;
          break;
        case SDL_CONTROLLER_AXIS_RIGHTX:
          joy.axes[2] = axis.second;
          break;
        case SDL_CONTROLLER_AXIS_RIGHTY:
          joy.axes[3] = axis.second;
          break;
        case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
          joy.axes[4] = axis.second * -1;
          break;
        case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
          joy.axes[5] = axis.second * -1;
          break;
        default:
          break;
      }
    }
    joy_pub_->publish(joy);
  }
}

}  // namespace playstation_controller_drivers

RCLCPP_COMPONENTS_REGISTER_NODE(playstation_controller_drivers::DualsenseDriverComponent)
