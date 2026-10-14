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
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
# ONNX Runtime for method B: /opt/onnxruntime of the dev image (docker/Dockerfile), not a rosdep key.
find_package(onnxruntime REQUIRED)

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
  message_filters tf2_ros tf2_eigen ament_index_cpp)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen opencv_core onnxruntime::onnxruntime)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(perception_node src/main.cpp)
target_link_libraries(perception_node ${PROJECT_NAME}_core)
target_compile_options(perception_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
# models/perception_cnn.onnx: the trained model of method B, the default of learned.model_path.
install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_node test/test_perception_node.cpp)
  target_link_libraries(test_perception_node ${PROJECT_NAME}_core)
  ament_add_gtest(test_grid_builder test/test_grid_builder.cpp)
  target_link_libraries(test_grid_builder ${PROJECT_NAME}_core)
  # Golden data of release_model.py; reads the model and the frame from the source tree.
  ament_add_gtest(test_cnn_classifier test/test_cnn_classifier.cpp)
  target_link_libraries(test_cnn_classifier ${PROJECT_NAME}_core opencv_imgcodecs opencv_imgproc)
  target_compile_definitions(test_cnn_classifier PRIVATE CAPSTONE_PERCEPTION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()

ament_package()
