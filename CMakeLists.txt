cmake_minimum_required(VERSION 3.8)
project(blimp_vision_disparity)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(
    ${PROJECT_NAME}_node
    src/blimp_vision_node.cpp
    src/BlimpVision.cpp
    src/ComputerVision.cpp
    include/BlimpVision.hpp
    include/ComputerVision.hpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
    std_msgs
    sensor_msgs
    camera_info_manager
    cv_bridge
    image_geometry
    image_transport
    OpenCV
)

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY param
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY calibration
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()