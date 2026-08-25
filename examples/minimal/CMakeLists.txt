cmake_minimum_required(VERSION 3.10)
project(minimal_alica CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Only three ALICA packages are needed for a non-ROS, statically-wired agent.
find_package(alica_solver_interface REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)

add_executable(minimal_alica
  src/main.cpp
  src/CountUp.cpp
)

target_link_libraries(minimal_alica PRIVATE
  alica_engine
  alica_dummy_proxy
  alica_solver_interface
)

target_include_directories(minimal_alica PRIVATE src)
