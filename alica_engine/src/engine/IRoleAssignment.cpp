#include "engine/IRoleAssignment.h"

#include "engine/AlicaEngine.h"

namespace alica
{
IRoleAssignment::IRoleAssignment()
        : ownRole(nullptr)
        , communication(nullptr)
{
}

const Role* IRoleAssignment::getRole(essentials::IdentifierConstPtr robotId)
{
    auto iter = this->robotRoleMapping.find(robotId);
    if (iter != this->robotRoleMapping.end()) {
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
    this->communication = communication;
}

} /* namespace alica */
