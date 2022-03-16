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
cmake -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true . && make -j $CPUS && make install

cd ~/temp
version=v1.6.0 && git clone --depth 1 -b $version https://github.com/opentracing/opentracing-cpp.git
cd opentracing-cpp/ && git submodule init && git submodule update
cmake -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true CMakeLists.txt && make -j $CPUS && make install

cd ~/temp
version=0.11.0 && git clone --depth 1 -b $version https://github.com/apache/thrift.git
cd thrift && ./bootstrap.sh && ./configure --with-nodejs=no --with-php=no --with-java=no --with-go=no --with-qt5=no --enable-tests=no --enable-tutorial=no && make -j $CPUS install

cd ~/temp
version=v0.7.0 && git clone --depth 1 -b $version https://github.com/jaegertracing/jaeger-client-cpp.git
cd jaeger-client-cpp/ && git submodule init && git submodule update
sed -i 's/Boost\ CONFIG/Boost/g' cmake/Config.cmake.in
cmake -DHUNTER_ENABLED=OFF -DBUILD_TESTING=OFF -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true CMakeLists.txt && make -j $CPUS && make install

cp ~/temp/jaeger-client-cpp/cmake/Findthrift.cmake /usr/local/lib/cmake/
rm -rf ~/temp/opentracing-cpp ~/temp/jaeger-client-cpp ~/temp/thrift ~/temp/json

ldconfig
