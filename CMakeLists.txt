cmake_minimum_required(VERSION 3.8)
project(data_provider_pkg)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

include_directories(include
        ${rclcpp_INCLUDE_DIRS}
        ${tf2_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${autoware_sensing_msgs_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${message_filters_INCLUDE_DIRS}
)

add_executable(data_provider_node
        src/data_provider_node.cpp)
ament_target_dependencies(data_provider_node rclcpp sensor_msgs nav_msgs
        tf2 tf2_ros message_filters geometry_msgs)
target_link_libraries(data_provider_node
        Eigen3::Eigen
        ${message_filter_LIBRARIES})

add_executable(lio_sam_imu_transform
        src/lio_sam_imu_transform.cpp)
ament_target_dependencies(lio_sam_imu_transform rclcpp sensor_msgs nav_msgs
        tf2 tf2_ros Eigen3 message_filters geometry_msgs)
target_link_libraries(lio_sam_imu_transform
        Eigen3::Eigen
        ${message_filter_LIBRARIES})

install(TARGETS data_provider_node lio_sam_imu_transform
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
