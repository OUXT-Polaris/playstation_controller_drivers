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
