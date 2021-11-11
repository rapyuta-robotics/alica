#pragma once

#include "engine/collections/RobotEngineData.h"
#include "model/Role.h"

#include <map>

namespace alica
{

class AlicaEngine;
class IAlicaCommunication;

class IRoleAssignment
{
public:
    IRoleAssignment();
    virtual ~IRoleAssignment() {}

    virtual void init() = 0;
    virtual void tick() = 0;
    virtual void update() = 0;

    const Role* getOwnRole() const { return _ownRole; }
    const Role* getRole(alica::AgentId agentId) const;
    void setCommunication(const IAlicaCommunication* communication);

protected:
    /**
     * Current Robot's role.
     */
    const Role* _ownRole;
    std::map<alica::AgentId, const Role*> _robotRoleMapping;
    const IAlicaCommunication* communication;
};
} // namespace alica
