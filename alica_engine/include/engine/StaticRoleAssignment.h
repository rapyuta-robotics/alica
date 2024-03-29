#pragma once

#include <engine/IRoleAssignment.h>

namespace alica
{
class PlanRepository;
class TeamManager;

class StaticRoleAssignment : public IRoleAssignment
{
public:
    StaticRoleAssignment(const IAlicaCommunication& communicator, const PlanRepository& planRepository, TeamManager& teamManager);
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
    const IAlicaCommunication& _communicator;
    const PlanRepository& _planRepository;
    TeamManager& _tm;
};

} /* namespace alica */
