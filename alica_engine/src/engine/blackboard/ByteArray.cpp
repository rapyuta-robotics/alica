
#include "engine/blackboard/ByteArray.h"

#include <assert.h>
#include <utility>
#include <string.h>

namespace alica {
ByteArray::ByteArray(const uint8_t* src, int32_t count)
    : _size(count) {
    assert(src != nullptr);
    if (count > 0) {
        _begin = static_cast<uint8_t*>(malloc(count));
        assert(_begin!=nullptr);
        memcpy(_begin, src, count);
    } else {
        _begin = nullptr;
    }
}

ByteArray::ByteArray(int32_t size)
    : _size(size)
{
    assert(size > 0);
    if(size> 0) {
        _begin = static_cast<uint8_t*>(malloc(size));
    } else {
        _begin = nullptr;
    }
}

ByteArray::ByteArray(ByteArray&& o)
    : _begin(o._begin)
    , _size(o._size) {
        o._begin = nullptr;
        //setting the size is not necessary
}

ByteArray& ByteArray::operator=(ByteArray&& o) {
    std::swap(_begin,o._begin);
    std::swap(_size,o._size);
    return *this;
}

}