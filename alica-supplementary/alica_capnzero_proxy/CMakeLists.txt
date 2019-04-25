cmake_minimum_required(VERSION 2.8.3)
project(alica_capnzero_proxy)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
    set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
    set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
    add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(Threads REQUIRED)

## ZMQ
find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ QUIET zmq)
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

find_package(CapnProto REQUIRED)


## Capnproto Messages:
message(STATUS "Capnproto message stuff")
set(capnp_message_folder "${CMAKE_CURRENT_SOURCE_DIR}/msgs")
file(GLOB_RECURSE capnp_messages ${capnp_message_folder} *.capnp)
# If the destination folder does not exist create it
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/msgs")
    message(STATUS "Destination folder for Capnproto compile results is not present")
    message(STATUS "Creating the folder automatically")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/msgs")
endif()

set(CAPNPC_SRC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/msgs)
set(CAPNPC_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/msgs")
message(STATUS "Compiling Capnproto messages")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})
add_library(alica_capnz_msgs ${CAPNP_SRCS})
#if (NOT catkin_FOUND)
#    target_include_directories(alica_capnz_LIBRARYS PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
#endif (NOT catkin_FOUND)


## Find required catkin packages:
find_package(catkin REQUIRED COMPONENTS
        alica_engine
        message_generation
        alica_msgs
        alica_common_config
        capnzero
        agent_id
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        LIBRARIES alica_capnzero_proxy
        CATKIN_DEPENDS alica_engine capnzero alica_msgs
        #DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CAPNP_INCLUDE_DIRS}
)

add_library(alica_capnzero_proxy
        src/communication/AlicaCapnzeroCommunication.cpp
        include/communication/AlicaCapnzeroCommunication.h
        include/msgs/AlicaEngineInfo.capnp.h
        )

add_dependencies(alica_capnzero_proxy
        alica_capnz_msgs
        )

target_compile_definitions(alica_capnzero_proxy PUBLIC ZMQ_BUILD_DRAFT_API=1)
target_link_libraries(alica_capnzero_proxy
        ${catkin_LIBRARIES}
        ${ZeroMQ_LIBRARY}
        ${CAPNP_LIBRARIES}
        alica_capnz_msgs
        )

add_executable(senderTest src/SenderTest.cpp)
target_link_libraries(senderTest
        alica_capnzero_proxy
        ${catkin_LIBRARIES}
        ${ZeroMQ_LIBRARY}
        ${CAPNP_LIBRARIES}

        )
target_compile_definitions(senderTest PUBLIC ZMQ_BUILD_DRAFT_API=1)

add_executable(recieverTest src/RecieverTest.cpp)
target_link_libraries(recieverTest
        alica_capnzero_proxy
        ${catkin_LIBRARIES}
        ${ZeroMQ_LIBRARY}
        ${CAPNP_LIBRARIES}
        alica_capnz_msgs
        )
target_compile_definitions(recieverTest PUBLIC ZMQ_BUILD_DRAFT_API=1)