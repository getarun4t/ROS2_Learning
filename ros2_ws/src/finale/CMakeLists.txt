cmake_minimum_required(VERSION 3.8)
project(finale)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# add_executable(finale src/turtle_controller.cpp)

#install(TARGETS
#  finale
#  DESTINATION lib/${PROJECT_NAME}/
#)

add_library(final_project_component SHARED src/turtle_controller.cpp)

ament_target_dependencies(final_project_component rclcpp rclcpp_lifecycle rclcpp_action rclcpp_components turtlesim geometry_msgs my_robot_interfaces)

rclcpp_components_register_nodes(final_project_component "finale::TurtleController")

add_library(turtle_bot_control_component SHARED src/turtle_bot_controller.cpp)

ament_target_dependencies(turtle_bot_control_component rclcpp rclcpp_lifecycle rclcpp_action rclcpp_components geometry_msgs my_robot_interfaces)

rclcpp_components_register_nodes(turtle_bot_control_component "finale::TurtleBotController")

install(TARGETS
  final_project_component
  turtle_bot_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
