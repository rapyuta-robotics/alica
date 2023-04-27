#!/bin/bash

GCCVERSION=$(gcc -dumpversion)
GPPVERSION=$(g++ -dumpversion)

if [[ ($GCCVERSION < 7) || ($GPPVERSION < 7) ]]; then
    printf "Minimum version not met for gcc & g++. Found gcc-$GCCVERSION g++-$GPPVERSION \nUpgrade to atleast gcc-7 & g++-7 or higher\n"
    exit 1
fi

# Prompt for sudo
[ "$UID" -eq 0 ] || exec sudo -E bash "$0" "$@"

apt update
apt-get install automake bison flex libtool libssl-dev libevent-dev libyaml-cpp-dev -y

CPUS=$(nproc)

mkdir -p ~/temp
cd ~/temp
version=v3.9.1 && git clone --depth 1 -b $version https://github.com/nlohmann/json.git
cd json
CXX=$CXX CC=$CC cmake -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true . && make -j $CPUS && make install

cd ~/temp
version=0.11.0 && git clone --depth 1 -b $version https://github.com/apache/thrift.git
cd thrift && CXX=$CXX CC=$CC ./bootstrap.sh && CXX=$CXX CC=$CC ./configure --with-nodejs=no --with-php=no --with-java=no --with-go=no --with-qt5=no --enable-tests=no --enable-tutorial=no && make -j $CPUS install

cd ~/temp
git clone https://github.com/google/benchmark.git
cd benchmark && cmake -E make_directory "build" && cmake -E chdir "build" cmake -DBENCHMARK_DOWNLOAD_DEPENDENCIES=on -DCMAKE_BUILD_TYPE=Release ../ && cmake --build "build" --config Release --target install

cd ~/temp
version=v1.13.0 git clone https://github.com/google/googletest.git -b $version
cd googletest && mkdir build && cd build && cmake .. -DCMAKE_CXX_STANDARD=17 && make && make install

cd ~/temp
curl https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protobuf-cpp-3.6.1.zip
unzip protobuf-cpp-3.6.1.zip -d .
cd protobuf-3.6.1 && cd cmake && mkdir build && cd build
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release .. && make install

cd ~/temp
git clone -b v1.54.0 https://github.com/grpc/grpc && cd grpc && git submodule update --init && ./test/distrib/cpp/run_distrib_test_cmake.sh

cd ~/temp
version=v1.9.0 && git clone --recurse-submodules -b $version https://github.com/open-telemetry/opentelemetry-cpp
cd opentelemetry-cpp && CXX=$CXX CC=$CC && mkdir build && cd build && 
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true -DWITH_JAEGER:BOOL=true -DWITH_STL:BOOL=true && cmake --build . --target all && cmake --install .
export LD_LIBRARY_PATH=/home/rr/catkin_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/opt/ortools/lib:/usr/lib:/usr/local/lib

rm -rf ~/temp/grpc ~/temp/protobuf-3.6.1 ~/temp/googletest ~/temp/benchmark ~/temp/opentelemetry ~/temp/thrift ~/temp/json

ldconfig