cmake_minimum_required(VERSION 3.8)
project(rm_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(DEBUG_MODE "option for debug" OFF)
if(DEBUG_MODE)
 add_definitions(-DDEBUG_MODE)
endif(DEBUG_MODE)
# set(CMAKE_BUILD_TYPE "Not Debug")
# if(CMAKE_BUILD_TYPE MATCHES "Debug")
#  add_definitions("-DDEBUG_MODE")
# endif()


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

#  src/TEST_mode_get_client.cpp
#  src/TEST_subscribtion_node.cpp

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

# #server test入口
# add_executable(get_mode_client src/TEST_mode_get_client.cpp)
# target_link_libraries(get_mode_client ${PROJECT_NAME})

# #subscriber test入口
# add_executable(subscriber src/TEST_subscribtion_node.cpp)
# target_link_libraries(subscriber ${PROJECT_NAME})

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

# #test server/subscriber
# install(TARGETS get_mode_client
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(TARGETS subscriber
#  DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} "rm_base::SimpleRobotBaseNode")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
