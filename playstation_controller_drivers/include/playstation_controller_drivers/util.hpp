#ifndef PLAYSTATION_CONTROLLER_DRIVERS__UTIL_HPP_
#define PLAYSTATION_CONTROLLER_DRIVERS__UTIL_HPP_

#include <boost/algorithm/clamp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <SDL2/SDL_gamecontroller.h>

namespace playstation_controller_drivers
{
struct LEDColor
{
  const uint8_t r;
  const uint8_t g;
  const uint8_t b;
  LEDColor(const std_msgs::msg::ColorRGBA & color)
  : r(static_cast<uint8_t>(boost::algorithm::clamp(color.r * 255, 0, 255))),
    g(static_cast<uint8_t>(boost::algorithm::clamp(color.g * 255, 0, 255))),
    b(static_cast<uint8_t>(boost::algorithm::clamp(color.b * 255, 0, 255)))
  {
  }
  LEDColor(const std_msgs::msg::ColorRGBA::SharedPtr color)
  : r(static_cast<uint8_t>(boost::algorithm::clamp(color->r * 255, 0, 255))),
    g(static_cast<uint8_t>(boost::algorithm::clamp(color->g * 255, 0, 255))),
    b(static_cast<uint8_t>(boost::algorithm::clamp(color->b * 255, 0, 255)))
  {
  }
};
}  // namespace playstation_controller_drivers

#endif  // PLAYSTATION_CONTROLLER_DRIVERS__UTIL_HPP_
