cmake_minimum_required(VERSION 3.10)
project(alica_customer_library)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_flags/cflags.cmake)
alica_compile_flags()

find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost required.\n")
endif (NOT Boost_FOUND)

find_package(alica_solver_interface REQUIRED)
find_package(alica_engine REQUIRED)

# ##########
# # Build ##
# ##########
set(alica_customer_library_SOURCES
  src/AcmeBehaviour.cpp
  src/AcmePlan.cpp
  src/AcmeRuntimeCondition.cpp
  src/WaitBehaviour.cpp
)

set(alica_customer_library_HEADER
  include/AcmeBehaviour.h
  include/AcmePlan.h
  include/AcmeRuntimeCondition.h
  include/WaitBehaviour.h
)

add_library(alica_customer_library SHARED
  ${alica_customer_library_SOURCES};
  ${alica_customer_library_HEADER};
)
target_link_libraries(alica_customer_library PUBLIC
                      alica_engine
                      alica_solver_interface
                      Boost::system
                      Boost::filesystem
)

target_include_directories(alica_customer_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")



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
