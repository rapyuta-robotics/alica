cmake_minimum_required(VERSION 3.5.1)
project(alica_turtlesim_library)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake_flags/cflags.cmake)
alica_compile_flags()

file(GLOB_RECURSE sources "src/*.cpp")

add_library(${PROJECT_NAME}
  ${sources};
)
target_link_libraries(${PROJECT_NAME} alica_engine constraintsolver)
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
