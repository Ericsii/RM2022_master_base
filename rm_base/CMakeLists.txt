cmake_minimum_required(VERSION 3.8)
project(rm_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include
include_directories(include)

# Create rm_base lib
add_library(${PROJECT_NAME} SHARED
  src/transporter_driver/uart_transporter.cpp
  src/simple_robot_base_node.cpp
)

set(dependencies
    rclcpp
    rclcpp_components
    rm_interfaces
    geometry_msgs
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})


# 创建入口
# SimpelRobotBase入口
add_executable(simple_robot_base src/simple_robot_base_main.cpp)
target_link_libraries(simple_robot_base ${PROJECT_NAME})

#register component nodes
# rclcpp_components_register_node(${PROJECT_NAME} "rm_base::SimpleRobotBaseNode")

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
install(TARGETS simple_robot_base
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # add_subdirectory(test)
endif()

ament_package()
