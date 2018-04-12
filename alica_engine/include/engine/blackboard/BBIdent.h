
#pragma once

#include <stdint.h>

namespace alica {

class BBIdent {
    public:
    BBIdent() : _val(0) {}
    BBIdent(int64_t id) 
        : _val(id) {}
    int64_t getRaw() const {return val;}

    bool operator>(const BBIdent o) const {
        return _val > o._val;
    }
    bool operator<(const BBIdent o) const {
        return _val < o._val;
    }
    bool operator>=(const BBIdent o) const {
        return !(_val < o._val);
    }
    bool operator<=(const BBIdent o) const {
        return !(_val > o._val);
    }
    bool operator==(const BBIdent o) const {
        return (_val == o._val);
    }
    bool operator!=(const BBIdent o) const {
        return !(_val == o._val);
    }

    private:
    int64_t _val;
};

}