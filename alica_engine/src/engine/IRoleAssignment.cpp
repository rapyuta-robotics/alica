#include "engine/IRoleAssignment.h"

#include "engine/AlicaEngine.h"

namespace alica {
IRoleAssignment::IRoleAssignment(const AlicaEngine* engine)
        : ownRole(nullptr)
        , engine(engine)
        , robotRoleMapping(map<const supplementary::AgentID*, Role*, supplementary::AgentIDComparator>())
        , communication(nullptr) {}

const Role* IRoleAssignment::getOwnRole() {
    return ownRole;
}

const Role* IRoleAssignment::getRole(const supplementary::AgentID* robotId) {
    auto iter = this->robotRoleMapping.find(robotId);
    if (iter != this->robotRoleMapping.end()) {
        return iter->second;
    } else {
        stringstream ss;
        ss << "RA: There is no role assigned for robot: " << robotId << std::endl;
        this->engine->abort(ss.str());
        return nullptr;
    }
}

void IRoleAssignment::setCommunication(const IAlicaCommunication* communication) {
    this->communication = communication;
}

} /* namespace alica */
