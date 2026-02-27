cmake_minimum_required(VERSION 3.8)
project(motion_validation VERSION 2.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
        include
        include/${PROJECT_NAME}
        ${EIGEN3_INCLUDE_DIR}
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
find_package(Eigen3 REQUIRED)

set(SOURCES
        include/domain/parameter.hpp
        src/application/correction.cpp
        src/application/kalman.cpp
        src/application/vertification.cpp
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
        Eigen3
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

target_link_libraries(${EXEC_NAME} Eigen3::Eigen)

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
