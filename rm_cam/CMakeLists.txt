cmake_minimum_required(VERSION 3.8)
project(rm_cam)

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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)

# Include
include_directories(include)

set(cam_drivers
    "src/cam_driver/virtual_cam.cpp"
)

# 创建shared lib
add_library(${PROJECT_NAME} SHARED
    ${cam_drivers}
    "src/cam_server.cpp"
    "src/cam_client.cpp"
    "src/wrapper_client.cpp"
    "src/virtual_cam_node.cpp"
)

# 设置依赖
set(dependencies
    rclcpp
    rclcpp_components
    sensor_msgs
    cv_bridge
    message_filters
    OpenCV
    rm_interfaces
    camera_info_manager
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# 创建入口
# VirtualCamera可执行入口
add_executable(virtual_cam "src/virtual_cam_node_main.cpp")
target_link_libraries(virtual_cam ${PROJECT_NAME})

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} "rm_cam::VirtualCamNode")

# install inlude directories
install(
    DIRECTORY include/
    DESTINATION include
)

# install libraries
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# install executable entry
install(
    TARGETS virtual_cam
    DESTINATION lib/${PROJECT_NAME}
)

# install launch resource config directory
install(
    DIRECTORY launch resource config
    DESTINATION share/${PROJECT_NAME}
)

# test
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    # add_subdirectory(test)
endif()

ament_package()
