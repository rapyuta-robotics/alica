cmake_minimum_required(VERSION 3.10)
project(alica_turtlesim_library)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_flags/cflags.cmake)
alica_compile_flags()

set(PKG_DEPS
  roscpp
  turtlesim
  geometry_msgs
)



find_package(yaml-cpp REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(constraintsolver REQUIRED)
find_package(autodiff REQUIRED)
find_package(alica_engine REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

# ##########
# # Build ##
# ##########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE sources "src/*.cpp")

add_library(alica_turtlesim_library
  ${sources};
)
target_link_libraries(alica_turtlesim_library ${catkin_LIBRARIES} alica_solver_interface alica_engine constraintsolver autodiff)
target_include_directories(alica_turtlesim_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
