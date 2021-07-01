cmake_minimum_required(VERSION 3.5)
project(playstation_controller_drivers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(color_names REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sdl2_vendor REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(dualsense_driver_component src/dualsense_driver_component.cpp)
target_include_directories(dualsense_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  dualsense_driver_component
  "rclcpp"
  "rclcpp_components"
  "sdl2_vendor"
  "sensor_msgs"
  "color_names"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dualsense_driver_component PRIVATE "PLAYSTATION_CONTROLLER_DRIVERS_BUILDING_LIBRARY")

rclcpp_components_register_nodes(dualsense_driver_component
  "playstation_controller_drivers::DualsenseDriverComponent")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS dualsense_driver_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(dualsense_driver_node src/dualsense_driver_node.cpp)
target_include_directories(dualsense_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dualsense_driver_node dualsense_driver_component)

install(TARGETS dualsense_driver_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  dualsense_driver_component
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
