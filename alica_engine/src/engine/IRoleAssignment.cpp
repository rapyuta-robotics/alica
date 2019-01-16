#include "engine/IRoleAssignment.h"

#include "engine/AlicaEngine.h"

namespace alica
{
IRoleAssignment::IRoleAssignment()
        : _ownRole(nullptr)
        , _communication(nullptr)
{
}

const Role* IRoleAssignment::getRole(AgentIDConstPtr robotId)
{
    auto iter = _robotRoleMapping.find(robotId);
    if (iter != _robotRoleMapping.end()) {
        return iter->second;
    } else {
        std::stringstream ss;
        ss << "RA: There is no role assigned for robot: " << *robotId << std::endl;
        AlicaEngine::abort(ss.str());
        return nullptr;
    }
}

void IRoleAssignment::setCommunication(const IAlicaCommunication* communication)
{
    _communication = communication;
}

} /* namespace alica */
