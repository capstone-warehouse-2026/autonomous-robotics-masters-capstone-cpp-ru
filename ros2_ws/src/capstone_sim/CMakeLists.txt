cmake_minimum_required(VERSION 3.16)
project(capstone_sim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(geometry_msgs REQUIRED)

# webots_ros2_driver loads this library as a plugin named in resource/tiago_base.urdf.
pluginlib_export_plugin_description_file(webots_ros2_driver diff_drive.xml)
add_library(${PROJECT_NAME} SHARED src/diff_drive.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} pluginlib rclcpp webots_ros2_driver geometry_msgs)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_diff_drive_kinematics test/test_diff_drive_kinematics.cpp)
  target_include_directories(test_diff_drive_kinematics PRIVATE include)
endif()

ament_export_libraries(${PROJECT_NAME})
ament_package()
