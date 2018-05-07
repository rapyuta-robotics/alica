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
    IRoleAssignment();
    virtual ~IRoleAssignment() {}

    virtual void init() = 0;
    virtual void tick() = 0;
    virtual void update() = 0;

    const Role* getOwnRole() const { return ownRole; }
    const Role* getRole(const supplementary::AgentID* robotId);
    void setCommunication(const IAlicaCommunication* communication);

protected:
    /**
     * Current Robot's role.
     */
    const Role* ownRole;
    std::map<const supplementary::AgentID*, const Role*, supplementary::AgentIDComparator> robotRoleMapping;
    const IAlicaCommunication* communication;
};
}  // namespace alica
