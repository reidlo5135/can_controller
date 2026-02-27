cmake_minimum_required(VERSION 3.8)
project(can_manager VERSION 2.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(can_utils REQUIRED)
find_package(can_dbc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
        include
        include/${PROJECT_NAME}
)

set(DEPENDS
        rclcpp
        can_msgs
        can_utils
        can_dbc
        std_msgs
        sensor_msgs
)

set(SOURCES
        include/domain/parameter.hpp
        src/domain/vehicle.cpp
        src/application/c2r.cpp
        src/application/r2c.cpp
        src/presentation/node.cpp
        src/main.cpp
)

set(EXEC_NAME
        can_manager
)

add_executable(${EXEC_NAME}
        ${SOURCES}
)

ament_target_dependencies(${EXEC_NAME}
        ${DEPENDS}
)

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        ${EXEC_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
