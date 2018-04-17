#pragma once

#include "engine/blackboard/ByteArray.h"
#include "engine/blackboard/BBIdent.h"
#include "engine/blackboard/ByteArray.h"
#include "engine/util/HashFunctions.h"

#include <map>
#include <stdint.h>
#include <string.h>
#include <type_traits>

namespace alica {

class BlackBoard {
public:
    
    using IdType = BBIdent;
    using ObjectType = ByteArray;
    BlackBoard() = default;
    
    IdType registerValue(const int8_t* buffer, int len);
    IdType registerValue(const char* buffer, int len);
    
    template <class InputIt>
    IdType registerValue(InputIt begin, InputIt end);

    bool hasValue(IdType id) const {return _body.find(id) != _body.end();}

    const ObjectType& getValue(IdType id) const {
        return _body.find(id)->second;
    }
    void removeValue(IdType id) {_body.erase(_body.find(id));}


    bool empty() const {return _body.empty();}
    int size() const {return _body.size();}

    BlackBoard(const BlackBoard&) = delete;
    BlackBoard(BlackBoard&&) = delete;
    BlackBoard& operator&=(const BlackBoard& ) = delete;
    BlackBoard& operator&=(BlackBoard&& ) = delete;


private:

    std::map<IdType, ObjectType> _body;
};

template <class InputIt>
BlackBoard::IdType BlackBoard::registerValue(InputIt begin, InputIt end) {
    static_assert(std::is_pod<typename InputIt::value_type>::value, "Iterators must point to POD type.");
    const int32_t len = std::distance(begin, end) * sizeof(typename InputIt::value_type);
    ObjectType element(len);
    int i = 0;
    while(begin != end) {
        memcpy(element.begin()+i, &*begin, sizeof(typename InputIt::value_type));
        ++begin;
        i+=sizeof(typename InputIt::value_type);
    }
    IdType id(Hash64(element.begin(), element.size()));

    _body[id] = std::move(element);
    return id;
}

}