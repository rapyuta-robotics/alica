cmake_minimum_required(VERSION 3.5.1)
project(alica_turtlesim)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY etc
  DESTINATION share/alica_ros2_turtlesim/
)
