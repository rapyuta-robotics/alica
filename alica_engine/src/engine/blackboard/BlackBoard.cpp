

#include "engine/blackboard/BlackBoard.h"

namespace alica
{

BlackBoard::IdType BlackBoard::registerValue(const int8_t* buffer, int len)
{
    ObjectType element(buffer, len);
    IdType id(Hash64(element.begin(), element.size()));
    _body[id] = std::move(element);
    return id;
}

BlackBoard::IdType BlackBoard::registerValue(const char* buffer, int len)
{
    return registerValue(reinterpret_cast<const int8_t*>(buffer), len);
}

} // namespace alica