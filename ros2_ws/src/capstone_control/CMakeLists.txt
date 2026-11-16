cmake_minimum_required(VERSION 3.16)
project(capstone_control LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ONNX Runtime for method B (PPO): /opt/onnxruntime of the dev image (docker/Dockerfile), not a rosdep key.
find_package(onnxruntime REQUIRED)

# Control algorithms without ROS: path tracking, controllers, limiter, goal/progress checks.
# Shared by the node, the tests and later the MPC/PPO controllers (docs/adr/0004).
add_library(${PROJECT_NAME}_algorithms STATIC
  src/path_tracker.cpp
  src/pure_pursuit_controller.cpp
  src/command_limiter.cpp
  src/checkers.cpp)
set_target_properties(${PROJECT_NAME}_algorithms PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_algorithms PUBLIC include)
target_compile_options(${PROJECT_NAME}_algorithms PRIVATE -Wall -Wextra -Wpedantic)

# Node code lives in a library so that the executable and the tests share it.
add_library(${PROJECT_NAME}_core STATIC src/control_node.cpp)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
target_link_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_algorithms)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(control_node src/main.cpp)
target_link_libraries(control_node ${PROJECT_NAME}_core)
target_compile_options(control_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS control_node DESTINATION lib/${PROJECT_NAME})

# Network input and output of method B without ONNX Runtime: a copy of ml/control/spec.py.
add_library(${PROJECT_NAME}_ppo_io STATIC src/ppo_observation.cpp)
set_target_properties(${PROJECT_NAME}_ppo_io PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_ppo_io PUBLIC include)
target_link_libraries(${PROJECT_NAME}_ppo_io ${PROJECT_NAME}_algorithms)
target_compile_options(${PROJECT_NAME}_ppo_io PRIVATE -Wall -Wextra -Wpedantic)

# Method B (PPO): ONNX inference of the policy trained in ml/control.
add_library(${PROJECT_NAME}_ppo STATIC src/ppo_controller.cpp)
set_target_properties(${PROJECT_NAME}_ppo PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_ppo PUBLIC include)
target_link_libraries(${PROJECT_NAME}_ppo ${PROJECT_NAME}_ppo_io onnxruntime::onnxruntime)
target_compile_options(${PROJECT_NAME}_ppo PRIVATE -Wall -Wextra -Wpedantic)
# models/ppo_policy.onnx and its manifest appear after ml/control/export_onnx.py.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models)
  install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_node test/test_control_node.cpp)
  target_link_libraries(test_control_node ${PROJECT_NAME}_core)

  # Pure C++ tests of the algorithms; no ROS context needed.
  ament_add_gtest(test_control_algorithms
    test/test_path_tracker.cpp
    test/test_pure_pursuit.cpp
    test/test_limiter_and_checkers.cpp
    test/test_closed_loop.cpp)
  target_link_libraries(test_control_algorithms ${PROJECT_NAME}_algorithms)

  ament_add_gtest(test_ppo_observation test/test_ppo_observation.cpp)
  target_link_libraries(test_ppo_observation ${PROJECT_NAME}_ppo_io)
  # Golden data of ml/control/export_onnx.py; reads the model and the cases from the source tree.
  ament_add_gtest(test_ppo_controller test/test_ppo_controller.cpp)
  target_link_libraries(test_ppo_controller ${PROJECT_NAME}_ppo)
  target_compile_definitions(test_ppo_controller PRIVATE CAPSTONE_CONTROL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()

ament_package()
