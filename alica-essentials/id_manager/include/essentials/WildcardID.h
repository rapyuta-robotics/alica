#pragma once

#include "essentials/Identifier.h"

#include <iostream>

namespace essentials
{
class IDManager;
class WildcardID : public essentials::Identifier
{
public:
    friend IDManager;
    virtual ~WildcardID();

    std::string toString() const;
    std::size_t hash() const;
    bool operator==(const essentials::Identifier& obj) const;
    bool operator!=(const essentials::Identifier& obj) const;
    bool operator<(const essentials::Identifier& other) const;
    bool operator>(const essentials::Identifier& other) const;
private:
    WildcardID(const uint8_t* idBytes, int idSize);
};
} /* namespace essentials */
