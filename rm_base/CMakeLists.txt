cmake_minimum_required(VERSION 3.8)
project(rm_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE MATCHES "Debug")
 add_definitions("-DRM_DEBUG_MODE")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rm_util REQUIRED)

# Include
include_directories(include)

# Create rm_base lib
add_library(${PROJECT_NAME} SHARED
  src/transporter_driver/uart_transporter.cpp
  src/robot_base_node.cpp
)

set(dependencies
    rclcpp
    rclcpp_components
    rm_interfaces
    geometry_msgs
    std_msgs
    sensor_msgs
    rm_util
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})


# 创建入口
# RobotBase入口
add_executable(robot_base src/robot_base_main.cpp)
target_link_libraries(robot_base ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION lib
)
install(TARGETS robot_base
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} "rm_base::RobotBaseNode")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
