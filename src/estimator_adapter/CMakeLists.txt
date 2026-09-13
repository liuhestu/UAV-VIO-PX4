cmake_minimum_required(VERSION 3.8)
project(estimator_adapter)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(adapter_core src/adapter_core.cpp)
target_compile_features(adapter_core PUBLIC cxx_std_17)
target_include_directories(adapter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(adapter_core rclcpp nav_msgs geometry_msgs Eigen3)

add_executable(estimator_adapter_node src/estimator_adapter_node.cpp)
target_compile_features(estimator_adapter_node PUBLIC cxx_std_17)
target_link_libraries(estimator_adapter_node adapter_core)
ament_target_dependencies(estimator_adapter_node rclcpp nav_msgs)

install(TARGETS adapter_core estimator_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_adapter_core test/test_adapter_core.cpp)
  target_link_libraries(test_adapter_core adapter_core)
  ament_target_dependencies(test_adapter_core nav_msgs geometry_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(nav_msgs geometry_msgs Eigen3)
ament_package()
