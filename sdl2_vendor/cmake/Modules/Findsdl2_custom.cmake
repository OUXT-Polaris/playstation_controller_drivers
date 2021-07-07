# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(SDL2 QUIET)
if(SDL2_FOUND)
  # Some systems (like Ubuntu 18.04) provide CMake "old-style" SDL2_INCLUDE_DIRS
  # and SDL2_LIBRARIES.  In these cases, generate a fake SDL2::SDL2 so that
  # downstreams consumers can just target_link_libraries(SDL2::SDL2)
  if(NOT TARGET SDL2::SDL2)
    add_library(SDL2::SDL2 INTERFACE IMPORTED)
    set_property(TARGET SDL2::SDL2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${SDL2_INCLUDE_DIRS})
    target_link_libraries(SDL2::SDL2 INTERFACE ${SDL2_LIBRARIES})
  endif()
endif()
