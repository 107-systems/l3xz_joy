#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(l3xz_joy)
set(L3XZ_JOY_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
#######################################################################################
include_directories(
  include
)
#######################################################################################
add_executable(${L3XZ_JOY_TARGET}
  src/Joystick.cpp
  src/main.cpp
  src/JoystickNode.cpp
)
#######################################################################################
target_compile_features(${L3XZ_JOY_TARGET} PRIVATE cxx_std_17)
ament_target_dependencies(${L3XZ_JOY_TARGET} rclcpp sensor_msgs)
#######################################################################################
install(TARGETS
  ${L3XZ_JOY_TARGET}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
