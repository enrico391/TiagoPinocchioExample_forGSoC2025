cmake_minimum_required(VERSION 3.22)
project(pinocchio_tiago)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build and install the executable.
# NOTE: You must use `target_link_libraries` to get collision checking capabilities.
add_executable(pinocchio_tiago src/pinocchio_tiago.cpp)
ament_target_dependencies(pinocchio_tiago ament_index_cpp Eigen3 eigen3_cmake_module rclcpp std_msgs)
target_link_libraries(pinocchio_tiago pinocchio::pinocchio)

install(
  TARGETS
  pinocchio_tiago
  DESTINATION lib/pinocchio_tiago
)

# Install the robot models.
install(
  DIRECTORY srdf
  DESTINATION share/pinocchio_tiago
)

ament_package()