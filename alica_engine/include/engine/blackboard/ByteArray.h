#pragma once

#include <stdint.h>
#include <stdlib.h>

namespace alica {

class ByteArray {
    public:
    ByteArray()
        : _begin(nullptr)
        , _size(0) {}
    ByteArray(const int8_t* src, int32_t size);
    ByteArray(int32_t size);
    ~ByteArray() {
        free(_begin);
    }
    int32_t size() const {return _size;}
    const int8_t* begin() const {return _begin;}
    const int8_t* end() const {return _begin+_size;}
    int8_t* begin() {return _begin;}
    int8_t* end() {return _begin+_size;}
    bool empty() const {return _size==0;}
    ByteArray(ByteArray&& o);
    ByteArray& operator=(ByteArray&& o);
    
    ByteArray(const ByteArray& o)=delete;
    ByteArray& operator=(const ByteArray& o)=delete;
    
    private:
    int8_t* _begin;
    int32_t _size;
};
}