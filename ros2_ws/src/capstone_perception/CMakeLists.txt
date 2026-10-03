cmake_minimum_required(VERSION 3.16)
project(capstone_perception LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

# Node, grid builder and both methods live in one library shared by the executable and the tests.
add_library(${PROJECT_NAME}_core STATIC
  src/perception_node.cpp
  src/grid_builder.cpp
  src/pixel_classifier.cpp
  src/geometric_classifier.cpp   # method A, participant 1
  src/cnn_classifier.cpp         # method B, participant 2
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp sensor_msgs nav_msgs diagnostic_msgs cv_bridge
  message_filters tf2_ros tf2_eigen)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen ${OpenCV_LIBS})
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(perception_node src/main.cpp)
target_link_libraries(perception_node ${PROJECT_NAME}_core)
target_compile_options(perception_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_node test/test_perception_node.cpp)
  target_link_libraries(test_perception_node ${PROJECT_NAME}_core)
  ament_add_gtest(test_grid_builder test/test_grid_builder.cpp)
  target_link_libraries(test_grid_builder ${PROJECT_NAME}_core)
endif()

ament_package()
