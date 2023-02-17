cmake_minimum_required(VERSION 3.5.1)
project(libalica-supplementary-tests)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake_flags/cflags.cmake)
alica_compile_flags()

add_library(${PROJECT_NAME} SHARED
  src/libalica-supplementary-tests/AcmeBehaviour.cpp
  src/libalica-supplementary-tests/AcmePlan.cpp
  src/libalica-supplementary-tests/AcmeRuntimeCondition.cpp
  src/libalica-supplementary-tests/WaitBehaviour.cpp
  src/libalica-supplementary-tests/conditions.cpp
  src/libalica-supplementary-tests/GSolver/SolverTestBehaviour.cpp
  src/libalica-supplementary-tests/ProblemModule/QueryBehaviour1.cpp
)
target_link_libraries(${PROJECT_NAME} alica_engine)
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
