cmake_minimum_required(VERSION 3.5.1)
project(alica_turtlesim_library)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake_flags/cflags.cmake)
alica_compile_flags()

find_package(yaml-cpp REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(constraintsolver REQUIRED)
find_package(autodiff REQUIRED)
find_package(alica_engine REQUIRED)

# ##########
# # Build ##
# ##########
file(GLOB_RECURSE sources "src/*.cpp")

add_library(${PROJECT_NAME}
  ${sources};
)
target_link_libraries(${PROJECT_NAME} alica_solver_interface alica_engine constraintsolver autodiff)
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

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
