#pragma once

#include <map>

class BlackBoard {
    using ObjectType = std::vector<uint8_t>;
    using IdType = int64_t;

    std::map<IdType,ObjectType> _body;
};