cmake_minimum_required(VERSION 3.5)
project(ros2_websocket_proxy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(websocketpp REQUIRED)
find_package(ament_index_cpp REQUIRED)


include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

ament_export_dependencies(rclcpp yaml-cpp ament_index_cpp)

add_executable(generic_client src/client.cpp)
ament_target_dependencies(generic_client rclcpp rclcpp_components yaml-cpp websocketpp ament_index_cpp)

add_executable(generic_server src/server.cpp)
ament_target_dependencies(generic_server rclcpp rclcpp_components yaml-cpp websocketpp ament_index_cpp)

target_link_libraries(generic_client
  ${YAML_CPP_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

target_link_libraries(generic_server
  ${YAML_CPP_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

install(TARGETS
    generic_client
    generic_server
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
