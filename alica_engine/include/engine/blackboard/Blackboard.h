#pragma once

#include "engine/blackboard/ByteArray.h"

#include <map>


class BlackBoard {
    public:
    
    using IdType = BBIdent;
    using ObjecType = ByteArray;
    BlackBoard();

    //IdType registerValue(uint8_t be)



    BlackBoard(const BlackBoard&) = delete;
    BlackBoard(BlackBoard&&) = delete;
    BlackBoard& operator&=(const BlackBoard& ) = delete;
    BlackBoard& operator&=(BlackBoard&& ) = delete;




    std::map<IdType,ObjectType> _body;
};