#pragma once

#include "supplementary/AgentID.h"

#include "model/Role.h"
#include "engine/collections/RobotEngineData.h"

#include <map>

namespace alica {

class AlicaEngine;
class IAlicaCommunication;

class IRoleAssignment {
public:
    IRoleAssignment(const AlicaEngine* ae);
    virtual ~IRoleAssignment() {}

    virtual void init() = 0;
    virtual void tick() = 0;
    virtual void update() = 0;

    const Role* getOwnRole();
    const Role* getRole(const supplementary::AgentID* robotId);
    void setCommunication(const IAlicaCommunication* communication);

protected:
    const AlicaEngine* engine;
    /**
     * Current Robot's role.
     */
    Role* ownRole;
    std::map<const supplementary::AgentID*, Role*, supplementary::AgentIDComparator> robotRoleMapping;
    const IAlicaCommunication* communication;
};
}  // namespace alica
