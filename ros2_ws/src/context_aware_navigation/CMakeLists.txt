cmake_minimum_required(VERSION 3.8)
project(context_aware_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(multi_person_tracker_interfaces)
find_package(sensor_msgs)

# Include cpp "include" directory
include_directories(include)

# Create cpp executables
# add_executable(cpp_exe src/cpp_node.cpp)
# ament_target_dependencies(cpp_exe rclcpp)

#install cpp executable
# install(TARGETS
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  scripts/social_map_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
