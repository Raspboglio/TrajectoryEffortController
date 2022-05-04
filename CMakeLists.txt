cmake_minimum_required(VERSION 3.5)
project(trajectory_effort_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(realtime_tools REQUIRED)


add_library(pid_lib SHARED
  include/external/PID/cpp/PID.cpp  
)

add_library(trajectory_effort_controller SHARED
  src/trajectory_effort_controller.cpp
)

target_include_directories(trajectory_effort_controller PRIVATE include)
ament_target_dependencies(trajectory_effort_controller 
  rclcpp 
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  trajectory_msgs
)
target_link_libraries(trajectory_effort_controller pid_lib)

target_compile_definitions(trajectory_effort_controller PRIVATE "TRAJECTORY_EFFORT_CONTROLLER_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface trajectory_effort_controller.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS trajectory_effort_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)

  
endif()



ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  trajectory_effort_controller
)

ament_package()
