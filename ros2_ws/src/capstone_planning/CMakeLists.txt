cmake_minimum_required(VERSION 3.16)
project(capstone_planning LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Node code lives in a library so that the executable and the tests share it.
add_library(${PROJECT_NAME}_core STATIC src/planning_node.cpp)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(planning_node src/main.cpp)
target_link_libraries(planning_node ${PROJECT_NAME}_core)
target_compile_options(planning_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS planning_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planning_node test/test_planning_node.cpp)
  target_link_libraries(test_planning_node ${PROJECT_NAME}_core)
endif()

ament_package()
