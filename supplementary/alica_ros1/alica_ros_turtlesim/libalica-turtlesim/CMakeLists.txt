cmake_minimum_required(VERSION 3.5.1)
project(libalica-turtlesim)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake_flags/cflags.cmake)
alica_compile_flags()

add_library(${PROJECT_NAME} SHARED
  src/CircleRuntimeCondition.cpp
  src/Idle.cpp
  src/GoTo.cpp
  src/GoToCalculatedResult.cpp
  src/MakeFormation.cpp
  src/RandomMoveTutorial.cpp
  src/Simulation.cpp
  src/SpawnTurtle.cpp
  src/SurroundLeaderTutorial.cpp
  src/Teleport.cpp
  src/TeleportToRandomPosition.cpp
  src/turtle.cpp
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
