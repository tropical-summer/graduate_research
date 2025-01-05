cmake_minimum_required(VERSION 3.12)
project(publisher_node)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(node_options REQUIRED)

set(LIBRARY_NAME ${PROJECT_NAME})
set(LIBRARY_DEPENDENCIES
    rclcpp
    std_msgs
    node_options
)

add_executable(${LIBRARY_NAME} src/${LIBRARY_NAME}.cpp)

target_include_directories(
    ${LIBRARY_NAME} 
    PUBLIC
    ${node_options_INCLUDE_DIRS}
  
)

ament_target_dependencies(${LIBRARY_NAME} ${LIBRARY_DEPENDENCIES})

install(TARGETS
    ${LIBRARY_NAME}
    DESTINATION lib/${LIBRARY_NAME}
)

ament_package()