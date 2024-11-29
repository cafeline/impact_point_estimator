cmake_minimum_required(VERSION 3.8)
project(impact_point_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(impact_point_estimator
  src/impact_point_estimator.cpp
)

ament_export_include_directories(include)
ament_export_libraries(impact_point_estimator)

ament_auto_package(INSTALL_TO_SHARE)