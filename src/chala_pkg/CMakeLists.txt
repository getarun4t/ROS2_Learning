cmake_minimum_required(VERSION 3.8)
project(chala_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(chala_cpp_node src/first_node.cpp)
ament_target_dependencies(chala_cpp_node rclcpp)

add_executable(robots_news_station src/robot_news_station.cpp)
ament_target_dependencies(robots_news_station rclcpp example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

install(TARGETS
  chala_cpp_node
  robots_news_station
  smartphone
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
