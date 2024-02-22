cmake_minimum_required(VERSION 3.8)
project(spot_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
	DESTINATION share/${PROJECT_NAME}
)

add_executable(republish_camera_info src/republish_camera_info.cpp)
ament_target_dependencies(republish_camera_info rclcpp sensor_msgs)

add_executable(publish_full_pointcloud src/publish_full_pointcloud.cpp)
target_link_libraries (publish_full_pointcloud ${PCL_LIBRARIES})
ament_target_dependencies(publish_full_pointcloud rclcpp sensor_msgs pcl_conversions tf2 tf2_ros tf2_sensor_msgs)

install(TARGETS
  republish_camera_info
  publish_full_pointcloud
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
