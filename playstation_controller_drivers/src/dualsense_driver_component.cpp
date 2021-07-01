#include <playstation_controller_drivers/dualsense_driver_component.hpp>
#include <playstation_controller_drivers/util.hpp>
#include <color_names/color_names.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace playstation_controller_drivers
{

DualsenseDriverComponent::DualsenseDriverComponent(const rclcpp::NodeOptions & options)
: Node("dualsense_driver", options)
{
  color_sub_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
    "~/color", 1,
    std::bind(&DualsenseDriverComponent::colorCallback, this, std::placeholders::_1));
  color_string_sub_ = this->create_subscription<std_msgs::msg::String>(
    "~/color/name", 1,
    std::bind(&DualsenseDriverComponent::colorNameCallback, this, std::placeholders::_1));
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

void DualsenseDriverComponent::getInput()
{
  while (rclcpp::ok()) {
    if (controller_ != nullptr) {
      SDL_Event e;
      SDL_WaitEventTimeout(&e, 10);
      if (e.type == SDL_JOYAXISMOTION) {
      } else if (e.type == SDL_CONTROLLERBUTTONDOWN) {
        handleJoyButtonDown(e);
      } else if (e.type == SDL_CONTROLLERBUTTONUP) {
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
      buttons_[SDL_CONTROLLER_BUTTON_DPAD_UP] = false;
      buttons_[SDL_CONTROLLER_BUTTON_DPAD_DOWN] = false;
      buttons_[SDL_CONTROLLER_BUTTON_DPAD_LEFT] = false;
      buttons_[SDL_CONTROLLER_BUTTON_DPAD_RIGHT] = false;
    }
  }
}

}  // namespace playstation_controller_drivers

RCLCPP_COMPONENTS_REGISTER_NODE(playstation_controller_drivers::DualsenseDriverComponent)
