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
cd thrift && CXX=$CXX CC=$CC ./bootstrap.sh &&
CXX=$CXX CC=$CC ./configure --with-nodejs=no --with-php=no --with-java=no --with-go=no --with-qt5=no --enable-tests=no --enable-tutorial=no &&
make -j $CPUS install

cd ~/temp
git clone https://github.com/google/benchmark.git
cd benchmark && cmake -E make_directory "build" &&
cmake -E chdir "build" cmake -DBENCHMARK_DOWNLOAD_DEPENDENCIES=on -DCMAKE_BUILD_TYPE=Release ../ &&
cmake --build "build" --config Release --target install

# This installs googletest with C++17 which is necessary for some packages
cd ~/temp
git clone https://github.com/google/googletest.git -b v1.13.0
cd googletest && mkdir build && cd build &&
cmake .. -DCMAKE_CXX_STANDARD=17 &&
make &&
make install

# This installs grpc and its dependencies inside the `build` folder in grpc
# This is extremely important because this is what makes us to isolate Protobuf and not expose this version
# to the simulation packages
cd ~/temp
git clone -b v1.54.0 git@github.com:rogeriofonteles/grpc.git && cd grpc && git submodule update --init &&
mkdir -p cmake/build && cd cmake/build &&
cmake -DCMAKE_BUILD_TYPE=Release -DgRPC_INSTALL=ON -DgRPC_SSL_PROVIDER=package -DgRPC_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=.  ../..

#This install opentelemetry from our forked repo of opentelemetry
cd ~/temp
git clone --recurse-submodules -b v1.9.0 git@github.com:rogeriofonteles/opentelemetry-cpp.git
cd opentelemetry-cpp && mkdir build && cd build &&
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true -DWITH_OTLP:BOOL=true -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=ON \
         -DWITH_STL:BOOL=true -DBUILD_TESTING:BOOL=false -DWITH_EXAMPLES:BOOL=true &&
cmake --build . --target all &&
cmake --install .

rm -rf ~/temp/thrift ~/temp/json

ldconfig
