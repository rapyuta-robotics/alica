#pragma once

#include "engine/collections/RobotEngineData.h"
#include "model/Role.h"

#include <essentials/IdentifierConstPtr.h>

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
    const Role* getRole(essentials::IdentifierConstPtr agentId) const;
    void setCommunication(const IAlicaCommunication* communication);

protected:
    /**
     * Current Robot's role.
     */
    const Role* _ownRole;
    std::map<essentials::IdentifierConstPtr, const Role*> _robotRoleMapping;
    const IAlicaCommunication* communication;
};
} // namespace alica
