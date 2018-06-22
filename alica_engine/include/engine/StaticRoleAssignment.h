#pragma once

#include <engine/IRoleAssignment.h>
#include <list>
#include <memory>

namespace alica
{
class AlicaEngine;
class TeamObserver;
class RobotProperties;

class StaticRoleAssignment : public IRoleAssignment
{
public:
    StaticRoleAssignment(AlicaEngine* ae);
    ~StaticRoleAssignment() = default;

    void init();
    void tick();
    void setCommunication(IAlicaCommunication* communication);
    void update();

    /**
     * Calculates the actual role assignment and is triggered if an
     * update is deemed necessary.
     */
    void calculateRoles();

private:
    bool updateRoles;

    AlicaEngine* ae;
};

} /* namespace alica */
