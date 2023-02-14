cmake_minimum_required(VERSION 3.5.1)
project(libalica-tests)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake_flags/cflags.cmake)
alica_compile_flags()

add_library(${PROJECT_NAME} SHARED
  src/libalica-tests/behaviours/AlwaysFail.cpp
  src/libalica-tests/behaviours/Attack.cpp
  src/libalica-tests/behaviours/AttackOpp
  src/libalica-tests/behaviours/BehAAA.cpp
  src/libalica-tests/behaviours/BehBAA.cpp
  src/libalica-tests/behaviours/ConstraintUsingBehaviour.cpp
  src/libalica-tests/behaviours/CountIndefinitely.cpp
  src/libalica-tests/behaviours/DefendMid.cpp
  src/libalica-tests/behaviours/EmptyBehaviour.cpp
  src/libalica-tests/behaviours/MidFieldStandard.cpp
  src/libalica-tests/behaviours/NotToTrigger.cpp
  src/libalica-tests/behaviours/ReadConfigurationBehaviour.cpp
  src/libalica-tests/behaviours/SuccessOnInitBeh.cpp
  src/libalica-tests/behaviours/SuccessSpam.cpp
  src/libalica-tests/behaviours/Tackle.cpp
  src/libalica-tests/behaviours/TestInheritBlackboardBehaviour.cpp
  src/libalica-tests/behaviours/TestParameterPassingBehaviour.cpp
  src/libalica-tests/behaviours/TriggerA.cpp
  src/libalica-tests/behaviours/TriggerB.cpp
  src/libalica-tests/behaviours/TriggerC.cpp
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
