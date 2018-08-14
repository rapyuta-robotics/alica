#pragma once

#include <engine/IRoleAssignment.h>

namespace alica
{
class AlicaEngine;

class StaticRoleAssignment : public IRoleAssignment
{
public:
    StaticRoleAssignment(AlicaEngine* ae);
    ~StaticRoleAssignment() = default;

    void init() override;
    void tick() override;
    void update() override;

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
