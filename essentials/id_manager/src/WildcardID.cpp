#include "essentials/WildcardID.h"
#include <typeinfo>

namespace essentials
{

WildcardID::WildcardID(const uint8_t* idBytes, int idSize)
        : essentials::Identifier(idBytes, idSize, WILDCARD_TYPE)
{
}

WildcardID::~WildcardID() {}

bool WildcardID::operator==(const essentials::Identifier& other) const
{
    try {
        dynamic_cast<const WildcardID&>(other);
    } catch (const std::bad_cast& e) {
        return false;
    }

    return true;
}

bool WildcardID::operator!=(const essentials::Identifier& other) const
{
    try {
        dynamic_cast<const WildcardID&>(other);
    } catch (const std::bad_cast& e) {
        return true;
    }

    return false;
}

bool WildcardID::operator<(const essentials::Identifier& other) const
{
    return true;
}

bool WildcardID::operator>(const essentials::Identifier& other) const
{
    return false;
}

std::string WildcardID::toString() const
{
    return "WildcardID (0)";
}

std::size_t WildcardID::hash() const
{
    return 0;
}

} /* namespace essentials*/