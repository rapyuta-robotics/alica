

#include "engine/blackboard/BlackBoard.h"

namespace alica
{
void BlackBoard::registerValue(const std::string& key, std::any val)
{
    vals.emplace(key, val);
}

void BlackBoard::registerValue(const char* key, std::any val)
{
    vals.emplace(key, val);
}

} // namespace alica