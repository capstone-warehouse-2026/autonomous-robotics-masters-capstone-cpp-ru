cmake_minimum_required(VERSION 3.16)
project(capstone_integration LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)

# Pure logic without ROS: deterministic unit tests keep the maths cheap to verify.
add_library(${PROJECT_NAME}_logic STATIC
  src/kinematics.cpp
  src/path_builder.cpp
  src/episode_record.cpp
  src/contracts.cpp
  src/audit_core.cpp
)
set_target_properties(${PROJECT_NAME}_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_logic PUBLIC include)
target_compile_options(${PROJECT_NAME}_logic PRIVATE -Wall -Wextra -Wpedantic)

# Node code in a library so executables and tests share it.
add_library(${PROJECT_NAME}_core STATIC
  src/world_stub_node.cpp
  src/path_stub_node.cpp
  src/drive_stub_node.cpp
  src/mission_stub_node.cpp
  src/graph_audit_node.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp rclcpp_action geometry_msgs nav_msgs sensor_msgs diagnostic_msgs tf2 tf2_ros nav2_msgs)
target_link_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_logic)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

foreach(node world_stub path_stub drive_stub mission_stub)
  add_executable(${node}_node src/${node}_main.cpp)
  target_link_libraries(${node}_node ${PROJECT_NAME}_core)
  target_compile_options(${node}_node PRIVATE -Wall -Wextra -Wpedantic)
  install(TARGETS ${node}_node DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(graph_audit src/graph_audit_main.cpp)
target_link_libraries(graph_audit ${PROJECT_NAME}_core)
target_compile_options(graph_audit PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS graph_audit DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME}_logic)

  ament_add_gtest(test_path_builder test/test_path_builder.cpp)
  target_link_libraries(test_path_builder ${PROJECT_NAME}_logic)

  ament_add_gtest(test_episode_record test/test_episode_record.cpp)
  target_link_libraries(test_episode_record ${PROJECT_NAME}_logic)

  ament_add_gtest(test_audit_core test/test_audit_core.cpp)
  target_link_libraries(test_audit_core ${PROJECT_NAME}_logic)

  ament_add_gtest(test_world_stub_node test/test_world_stub_node.cpp TIMEOUT 60)
  target_link_libraries(test_world_stub_node ${PROJECT_NAME}_core)
endif()

ament_package()
