#include "enigne/BlackBoard/ByteArray.cpp"

#include <assert.h>
#include <utility>

namespace alica {
ByteArray::ByteArray(uint8_t* src, int32_t size)
    : _size(size) {
    assert(src!= nullptr);
    if (size > 0) {
        _begin = static_cast<uint8_t>(malloc(size));
        assert(begin!=nullptr);
        memcpy(_begin,src, size);
    } else {
        _begin = nullptr;
    }
}

ByteArray::ByteArray(ByteArray&& o)
    : _begin(o._begin)
    , _size(o.size) {
        o._begin = nullptr;
        //setting the size is not necessary
}

ByteArray::ByteArray& operator=(ByteArray&& o);
    std::swap(_begin,o._begin);
    std::swap(_size,o._size);
    return *this;
}

}