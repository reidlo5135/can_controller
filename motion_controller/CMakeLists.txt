cmake_minimum_required(VERSION 3.5)
project(motion_controller VERSION 2.0.2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
        include
        include/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(can_utils REQUIRED)
find_package(can_dbc REQUIRED)
find_package(map_handle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(SOURCES
	src/domain/vehicle.cpp
        src/application/converter.cpp
	src/application/control.cpp
        src/application/motion.cpp
        src/presentation/node.cpp
        src/main.cpp
)

set(DEPENDS
        rclcpp
        std_msgs
        geometry_msgs
        can_msgs
        can_utils
        can_dbc
	map_handle_msgs
        sensor_msgs
)

set(EXEC_NAME
        ${PROJECT_NAME}
)

add_executable(${EXEC_NAME}
        ${SOURCES}
)

ament_target_dependencies(${EXEC_NAME}
        ${DEPENDS}
)

install(TARGETS
        ${EXEC_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
