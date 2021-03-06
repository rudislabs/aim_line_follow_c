cmake_minimum_required(VERSION 3.5)
project(aim_line_follow_c)

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
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nxp_cup_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(aim_line_follow_c src/aim_line_follow_c.cpp)
ament_target_dependencies(aim_line_follow_c rclcpp std_msgs geometry_msgs nxp_cup_interfaces)

install(TARGETS
  aim_line_follow_c
  DESTINATION lib/${PROJECT_NAME})

ament_package()
