cmake_minimum_required(VERSION 3.16)
project(capstone_fleet LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)

add_library(capstone_fleet_planning src/planner.cpp src/assignment.cpp)
target_include_directories(capstone_fleet_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(capstone_fleet_planning PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS capstone_fleet_planning EXPORT export_capstone_fleet
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_capstone_fleet HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fleet_planner test/test_planner.cpp)
  target_link_libraries(test_fleet_planner capstone_fleet_planning)
endif()
ament_package()
