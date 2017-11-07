#include "supplementary/BroadcastID.h"
#include <typeinfo>

namespace supplementary
{

BroadcastID::BroadcastID(const uint8_t *idBytes, int idSize)
    : id(0)
{
}

BroadcastID::~BroadcastID()
{
}

uint8_t *BroadcastID::getRaw() const
{
    return (uint8_t *)&this->id;
}

int BroadcastID::getSize() const
{
    return sizeof(uint8_t);
}

uint8_t BroadcastID::getType() const
{
	return BroadcastID::TYPE;
}

bool BroadcastID::operator==(const supplementary::IAgentID &other) const
{
    try
    {
        dynamic_cast<const BroadcastID &>(other);
    }
    catch (const std::bad_cast &e)
    {
        return false;
    }

    return true;
}

bool BroadcastID::operator!=(const supplementary::IAgentID &other) const
{
    try
    {
        dynamic_cast<const BroadcastID &>(other);
    }
    catch (const std::bad_cast &e)
    {
        return true;
    }

    return false;
}

bool BroadcastID::operator<(const supplementary::IAgentID &other) const
{
    return true;
}

bool BroadcastID::operator>(const supplementary::IAgentID &other) const
{
    return false;
}

std::vector<uint8_t> BroadcastID::toByteVector() const
{
    std::vector<uint8_t> bytes;
    bytes.push_back(this->id);
    return bytes;
}

std::string BroadcastID::toString() const
{
	return "BroadcastID (0)";
}

std::size_t BroadcastID::hash() const
{
	return 0;
}

} /* namespace supplementary*/

