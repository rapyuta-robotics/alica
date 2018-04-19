
#pragma once

#include <stdint.h>

namespace alica {

class BBIdent {
    public:
    BBIdent() = default;
    BBIdent(uint64_t id) 
        : _val(id) {}
    int64_t getRaw() const {return _val;}

    bool operator>(const BBIdent o) const {
        return _val > o._val;
    }
    bool operator<(const BBIdent o) const {
        return _val < o._val;
    }
    bool operator>=(const BBIdent o) const {
        return !(*this < o);
    }
    bool operator<=(const BBIdent o) const {
        return !(*this > o);
    }
    bool operator==(const BBIdent o) const {
        return (_val == o._val);
    }
    bool operator!=(const BBIdent o) const {
        return !(*this == o);
    }

    private:
    uint64_t _val;
};

}