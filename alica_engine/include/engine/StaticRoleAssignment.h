#pragma once

#include <engine/IRoleAssignment.h>

namespace alica
{
class AlicaEngine;

class StaticRoleAssignment : public IRoleAssignment
{
public:
    StaticRoleAssignment(const AlicaEngine* ae);
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
    bool _updateRoles;
    const AlicaEngine* _ae;
};

} /* namespace alica */
