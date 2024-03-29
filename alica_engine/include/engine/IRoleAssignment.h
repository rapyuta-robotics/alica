#pragma once

#include "engine/Types.h"
#include "engine/collections/RobotEngineData.h"
#include "model/Role.h"

#include <map>

namespace alica
{

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
    const Role* getRole(AgentId agentId) const;
    void setCommunication(const IAlicaCommunication* communication);

protected:
    static constexpr const char* LOGNAME = "RoleAssignment";

    /**
     * Current Robot's role.
     */
    const Role* _ownRole;
    std::map<AgentId, const Role*> _robotRoleMapping;
    const IAlicaCommunication* communication;
};
} // namespace alica
