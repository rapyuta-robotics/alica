cmake_minimum_required(VERSION 3.5.1)
project(alica-ros2-turtlesim)

include(../../../../cmake_flags/cflags.cmake)
alica_compile_flags()

find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/CircleRuntimeConditionConstraint.cpp
  src/conditions.cpp
  src/Go2RandomPosition.cpp
  src/GoTo.cpp
  src/Master.cpp
  src/Move.cpp
  src/turtle.cpp
  src/world_model.cpp
)
target_link_libraries(${PROJECT_NAME} alica_engine constraintsolver)
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
