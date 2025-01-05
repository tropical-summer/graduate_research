cmake_minimum_required(VERSION 3.12)
project(node_options)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(cxxopts REQUIRED)

set(LIBRARY_SRC
    src/cli_options.cpp
)
set(LIBRARY_DEPENDENCIES
    cxxopts
)
set(LIBRARY_NAME ${PROJECT_NAME})

add_library(${LIBRARY_NAME} SHARED ${LIBRARY_SRC})

target_include_directories(
    ${LIBRARY_NAME}
    PUBLIC
    include
)

ament_target_dependencies(${LIBRARY_NAME} ${LIBRARY_DEPENDENCIES})

install(TARGETS
    ${LIBRARY_NAME}
    DESTINATION lib/${LIBRARY_NAME}
)

ament_package()