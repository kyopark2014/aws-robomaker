cmake_minimum_required(VERSION 3.5)
project(jetbot_description)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)

ament_package()