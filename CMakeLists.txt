cmake_minimum_required(VERSION 3.8)
project(my_lifecycle_tutorial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lowercase_node src/lowercase_node.cpp)
ament_target_dependencies(lowercase_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
)

add_executable(uppercase_node src/uppercase_node.cpp)
ament_target_dependencies(uppercase_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
)

# 通常のNode
add_executable(reverse_node src/reverse_node.cpp)
ament_target_dependencies(reverse_node
  rclcpp
  std_msgs
)

# Supervisor (通常のNode)
add_executable(supervisor_node src/supervisor_node.cpp)
ament_target_dependencies(supervisor_node
  rclcpp
  lifecycle_msgs
)

install(TARGETS
  lowercase_node
  uppercase_node
  reverse_node
  supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# ---- GoogleTestのビルド設定 ----
ament_add_gtest(test_supervisor_component test/test_supervisor_component.cpp)
# 必要なライブラリ/インクルードを追加
target_include_directories(test_supervisor_component PRIVATE include)
target_link_libraries(test_supervisor_component
  # SupervisorComponent の静的関数ならライブラリなしでもOK
  # もしcomponent.cppをlibにまとめた場合は↓をアンコメント
  # my_lifecycle_tutorial_component
)
ament_target_dependencies(test_supervisor_component
  rclcpp
  # 他にcomponentで使っている依存があれば追記
)


ament_package()
