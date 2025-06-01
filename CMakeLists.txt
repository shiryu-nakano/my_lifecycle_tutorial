cmake_minimum_required(VERSION 3.8)
project(my_lifecycle_tutorial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ---- Componentライブラリ ----
add_library(supervisor_component src/supervisor_component.cpp)
target_include_directories(supervisor_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- Supervisor Node ----
add_executable(supervisor_node
  src/supervisor.cpp
  src/supervisor_node.cpp
)
target_include_directories(supervisor_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(supervisor_node
  rclcpp
  lifecycle_msgs
  nav_msgs
)


# ---- 他ノード ----
#add_executable(lowercase_node src/lowercase_node.cpp)
#target_include_directories(lowercase_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#ament_target_dependencies(lowercase_node
#  rclcpp
#  rclcpp_lifecycle
#  lifecycle_msgs
#  std_msgs
#)
#
#add_executable(uppercase_node src/uppercase_node.cpp)
#target_include_directories(uppercase_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#ament_target_dependencies(uppercase_node
#  rclcpp
#  rclcpp_lifecycle
#  lifecycle_msgs
#  std_msgs
#)
#
#add_executable(reverse_node src/reverse_node.cpp)
#target_include_directories(reverse_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#ament_target_dependencies(reverse_node
#  rclcpp
#  std_msgs
#)
#今は関係ないので一旦コメントアウト


# ---- GTest ----
ament_add_gtest(test_supervisor_component test/test_supervisor_component.cpp)
target_include_directories(test_supervisor_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_supervisor_component supervisor_component)
ament_target_dependencies(test_supervisor_component
  rclcpp
)

# ---- install ----
install(TARGETS
  #lowercase_node
  #uppercase_node
  #reverse_node
  supervisor_node
  supervisor_component
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
