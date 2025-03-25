cmake_minimum_required(VERSION 3.5)
project(b2_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
)

install(DIRECTORY
  launch
  config
  urdf
  model
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()