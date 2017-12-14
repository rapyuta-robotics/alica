#include "supplementary/BroadcastID.h"
#include <typeinfo>

namespace supplementary
{

BroadcastID::BroadcastID(const uint8_t *idBytes, int idSize)
    : AgentID(idBytes, idSize, BC_TYPE)
{
}

BroadcastID::~BroadcastID()
{
}

bool BroadcastID::operator==(const supplementary::AgentID &other) const
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

bool BroadcastID::operator!=(const supplementary::AgentID &other) const
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

bool BroadcastID::operator<(const supplementary::AgentID &other) const
{
    return true;
}

bool BroadcastID::operator>(const supplementary::AgentID &other) const
{
    return false;
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

