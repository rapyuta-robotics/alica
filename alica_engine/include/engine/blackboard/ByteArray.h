#pragma once
namespace alica {
class ByteArray {
    public:
    ByteArray()
        : _begin(nullptr)
        , _size(0)
    ByteArray(uint8_t* src, int32_t size);
    ~ByteArray() {
        free(_begin);
    }
    int32_t size() const {return _size;}
    const uint8_t* begin() const {return _begin;}
    const uint8_t* end() const {return _begin+_end;}
    bool empty() const {return _size==0;}
    ByteArray(ByteArray&& o);
    ByteArray& operator=(ByteArray&& o);
    
    ByteArray(const ByteArray& o)=delete;
    ByteArray& operator=(const ByteArray& o)=delete;
    
    private:
    uint8_t* _begin;
    int32_t _size
};
}