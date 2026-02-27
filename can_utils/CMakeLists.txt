cmake_minimum_required(VERSION 3.8)
project(can_utils VERSION 2.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_dbc REQUIRED)

include_directories(
        include
        include/${PROJECT_NAME}
)

set(SOURCES
        include/can_utils/callback.hpp
        include/can_utils/logger.hpp
        src/adaptor.cpp
        src/dump.cpp
        src/lib.cpp
        src/math.cpp
        src/send.cpp
)

set(DEPENDS
        rclcpp
        can_dbc
)

set(LIB_NAME ${PROJECT_NAME})

add_library(${LIB_NAME} SHARED
  ${SOURCES}
)

ament_target_dependencies(${LIB_NAME}
  ${DEPENDS}
)

install(TARGETS ${LIB_NAME}
  EXPORT canUtilsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${LIB_NAME})

ament_package()

install(EXPORT canUtilsTargets
  FILE canUtilsTargets.cmake
  NAMESPACE ${LIB_NAME}::
  DESTINATION lib/cmake/${LIB_NAME}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "canUtilsConfigVersion.cmake"
  VERSION 1.0
  COMPATIBILITY AnyNewerVersion
)

configure_file(canUtilsConfig.cmake.in canUtilsConfig.cmake @ONLY)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/canUtilsConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/canUtilsConfigVersion.cmake"
  DESTINATION lib/cmake/can_utils
)
