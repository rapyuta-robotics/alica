cmake_minimum_required(VERSION 3.10)
project(alica_solver_interface)

## Change to use clang ... unfortunately it doesnt work :(
#set(CMAKE_CXX_COMPILER "clang")
#
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++14" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake) 

# find_package(catkin QUIET)
# if (catkin_FOUND)
#   find_package(catkin REQUIRED)
  find_package(alica_common_config REQUIRED)
# else (catkin_FOUND)
#   set(catkin_LIBRARIES "alica_common_config")
# endif(catkin_FOUND)

# if(catkin_FOUND)
#   catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES alica_solver_interface
#   )
# endif(catkin_FOUND)

###########
## Build ##
###########

include_directories(include include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS} ${alica_common_config_INCLUDE_DIRS})

file(GLOB_RECURSE sources "src/*.cpp")


add_library(alica_solver_interface SHARED
    ${sources};
)

target_sources(alica_solver_interface PRIVATE
  ${sources})

  target_include_directories(alica_solver_interface PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
  )

target_link_libraries(alica_solver_interface PUBLIC)

# if (NOT catkin_FOUND)
 
# endif (NOT catkin_FOUND)


# if(CATKIN_ENABLE_TESTING)
#     catkin_add_gtest(${PROJECT_NAME}-test
#     test/test_intervals.cpp
#     test/test_main.cpp
#     )
#     if(TARGET ${PROJECT_NAME}-test)
#     target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
#     endif()
# else(CATKIN_ENABLE_TESTING)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

    find_package(GTest REQUIRED)
    include_directories(BEFORE
    ${GTEST_INCLUDE_DIRS}
    )
    add_executable(${PROJECT_NAME}-tests
    test/test_intervals.cpp
    test/test_main.cpp
    )
    target_link_libraries(${PROJECT_NAME}-tests PRIVATE Threads::Threads ${PROJECT_NAME} ${GTEST_LIBRARIES})
# endif(CATKIN_ENABLE_TESTING)

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

export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
 )
# exported targets
install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation})

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.0.0
  COMPATIBILITY SameMajorVersion)

  set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR)

  install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation})