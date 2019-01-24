#pragma once

#include "engine/AgentIDConstPtr.h"

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
    const Role* getRole(AgentIDConstPtr robotId);

protected:
    /**
     * Current Robot's role.
     */
    const Role* _ownRole;
    std::map<AgentIDConstPtr, const Role*> _robotRoleMapping;
};
} // namespace alica
