cmake_minimum_required(VERSION 3.16)
project(capstone_planning LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
# ONNX Runtime for method B: /opt/onnxruntime of the dev image (docker/Dockerfile), not a rosdep key.
find_package(onnxruntime REQUIRED)

# Method A without ROS: map fusion, inflation/cost, A*, path conversion and the synthetic scenarios.
# Shared by the node, the tests and planning_benchmark; method B reuses the same search and mask.
add_library(${PROJECT_NAME}_algorithms STATIC
  src/map_fusion.cpp
  src/cost_map.cpp
  src/grid_search.cpp
  src/planner.cpp
  src/scenarios.cpp)
set_target_properties(${PROJECT_NAME}_algorithms PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_algorithms PUBLIC include)
target_compile_options(${PROJECT_NAME}_algorithms PRIVATE -Wall -Wextra -Wpedantic)

# Method B: network input/output and ONNX Runtime inference of the learned cost (no ROS).
add_library(${PROJECT_NAME}_learned STATIC src/learned_cost.cpp)
set_target_properties(${PROJECT_NAME}_learned PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_learned PUBLIC include)
target_link_libraries(${PROJECT_NAME}_learned ${PROJECT_NAME}_algorithms onnxruntime::onnxruntime)
target_compile_options(${PROJECT_NAME}_learned PRIVATE -Wall -Wextra -Wpedantic)

# Node code lives in a library so that the executable and the tests share it.
add_library(${PROJECT_NAME}_core STATIC src/planning_node.cpp)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
target_link_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_algorithms ${PROJECT_NAME}_learned)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs
  diagnostic_msgs tf2 tf2_ros ament_index_cpp)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(planning_node src/main.cpp)
target_link_libraries(planning_node ${PROJECT_NAME}_core)
target_compile_options(planning_node PRIVATE -Wall -Wextra -Wpedantic)
add_executable(planning_benchmark src/planning_benchmark.cpp)
target_link_libraries(planning_benchmark ${PROJECT_NAME}_algorithms)
target_compile_options(planning_benchmark PRIVATE -Wall -Wextra -Wpedantic)
add_executable(planning_replay src/planning_replay.cpp)
target_link_libraries(planning_replay ${PROJECT_NAME}_learned)
target_compile_options(planning_replay PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS planning_node planning_benchmark planning_replay DESTINATION lib/${PROJECT_NAME})
# models/planning_cost.onnx: the trained model of method B, the default of learned.model_path.
install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planning_node test/test_planning_node.cpp TIMEOUT 120)
  target_link_libraries(test_planning_node ${PROJECT_NAME}_core)
  target_compile_definitions(test_planning_node PRIVATE CAPSTONE_PLANNING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  # Pure C++ tests of the algorithms; no ROS context needed.
  ament_add_gtest(test_planning_algorithms test/test_algorithms.cpp)
  target_link_libraries(test_planning_algorithms ${PROJECT_NAME}_algorithms)

  # Method B: input/output conventions, golden inference of the released model, fallback.
  ament_add_gtest(test_learned_cost test/test_learned_cost.cpp)
  target_link_libraries(test_learned_cost ${PROJECT_NAME}_learned)
  target_compile_definitions(test_learned_cost PRIVATE CAPSTONE_PLANNING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()

ament_package()
