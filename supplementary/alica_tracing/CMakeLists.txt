cmake_minimum_required(VERSION 3.5.1)
project(alica_tracing)
include(CTest)
add_compile_options(-std=c++17)


include(../../cmake_flags/cflags.cmake)
alica_compile_flags()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)

find_package(yaml-cpp REQUIRED)
find_package(alica_engine REQUIRED)

find_package(opentelemetry-cpp CONFIG REQUIRED)

set(CMAKE_SYSTEM_PREFIX_PATH "/opt/grpc/cmake/build/lib/cmake")
set(Protobuf_PROTOC_EXECUTABLE "/opt/grpc/cmake/build/bin/protoc")
set(Protobuf_INCLUDE_DIR "/opt/grpc/cmake/build/include")
set(Protobuf_LIBRARY "/opt/grpc/cmake/build/lib/libprotobuf.a")
set(Protobuf_PROTOC_LIBRARY "/opt/grpc/cmake/build/lib/libprotoc.a")
find_package(Protobuf REQUIRED)

set(CMAKE_SYSTEM_PREFIX_PATH "/opt/grpc/cmake/build/lib/cmake")
find_package(gRPC REQUIRED)

find_package(utf8_range CONFIG REQUIRED)
unset(CMAKE_SYSTEM_PREFIX_PATH CACHE)

###########
## Build ##
###########

# Declare a C++ library with project namespace to avoid naming collision
add_library(
  ${PROJECT_NAME} SHARED
  src/tracing/Trace.cpp
  src/tracing/TraceFactory.cpp
)

target_include_directories(alica_tracing PUBLIC
  ${alica_engine_INCLUDE_DIRS}
  ${OPENTELEMETRY_CPP_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME} PRIVATE
  ${YAML_CPP_LIBRARIES}
  "-Wl,-rpath=/usr/local/lib/,--disable-new-dtags"
  ${OPENTELEMETRY_CPP_LIBRARIES}
  utf8_range::utf8_validity
)

# if(BUILD_TESTING)    
#     find_package(GTest)
#     include(GoogleTest)
#     set(THREADS_PREFER_PTHREAD_FLAG ON)
#     find_package(Threads REQUIRED)
#     find_package(alica_solver_interface REQUIRED)

#     add_executable(${PROJECT_NAME}-test test/alica_tracing_test.cpp)

#     target_link_libraries(${PROJECT_NAME}-test
#       ${PROJECT_NAME}
#       ${alica_engine_LIBRARIES}
#       ${alica_solver_interface_LIBRARIES}
#       Threads::Threads
#       ${GTEST_LIBRARIES}
#     )

#     add_test(NAME TestAmrTracing.TestAmrTracingInit COMMAND ${PROJECT_NAME}-tests --gtest_filter=TestAmrTracing.TestAmrTracingInit)
# endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY include/
 DESTINATION include/
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Export targets to make package libraries
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
)

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)


# Installing files to make a find_package()-able package
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)
