#include "engine/IRoleAssignment.h"

#include "engine/AlicaEngine.h"

namespace alica
{
IRoleAssignment::IRoleAssignment()
        : _ownRole(nullptr)
        , communication(nullptr)
{
}

const Role* IRoleAssignment::getRole(AgentId robotId) const
{
    auto iter = _robotRoleMapping.find(robotId);
    if (iter != _robotRoleMapping.end()) {
        return iter->second;
    } else {
        AlicaEngine::abort(LOGNAME, "There is no role assigned for robot: ", robotId);
        return nullptr;
    }
}

} /* namespace alica */
