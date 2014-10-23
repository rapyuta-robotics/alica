
macro(rosbuild_make_udpProxy)

#set(CMAKE_CXX_FLAGS "-Wno-ignored-qualifiers")

rosbuild_find_ros_package("udp_proxy_generator")

execute_process(COMMAND mono ${udp_proxy_generator_PACKAGE_PATH}/bin/MakeUDPProxy.exe ${PROJECT_NAME})

#rosbuild_add_boost_directories()

FILE(GLOB FILES ${PROJECT_SOURCE_DIR}/proxy_gen/*.cpp)
rosbuild_add_executable(udpProxy ${FILES} ${outsources})
rosbuild_add_compile_flags(udpProxy -O3 -Wall -Werror)
rosbuild_link_boost(udpProxy system thread)
#rosbuild_link_boost(udpProxy asio thread system)
target_link_libraries(udpProxy castor)

endmacro(rosbuild_make_udpProxy)
