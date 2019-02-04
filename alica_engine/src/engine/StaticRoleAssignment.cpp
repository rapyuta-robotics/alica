//#define STATIC_RA_DEBUG

#include "engine/StaticRoleAssignment.h"
#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/collections/RobotProperties.h"
#include "engine/containers/RoleSwitch.h"
#include "engine/teammanager/TeamManager.h"

#include <engine/AgentIDConstPtr.h>

#include <alica_common_config/debug_output.h>

namespace alica
{

StaticRoleAssignment::StaticRoleAssignment(const AlicaEngine* ae)
        : IRoleAssignment()
        , _ae(ae)
        , _updateRoles(false)
{
}

/**
 * Initially calculates the robot-role mapping once.
 */
void StaticRoleAssignment::init()
{
    calculateRoles();
}

/**
 * Triggers the recalculation of the robot-role mapping, if the updateRoles flag is set to true.
 */
void StaticRoleAssignment::tick()
{
    if (_updateRoles) {
        _updateRoles = false;
        calculateRoles();
    }
}

/**
 * Sets the updateRoles flag to true, in order to recalculate the robot-role mapping on the next tick.
 */
void StaticRoleAssignment::update()
{
    _updateRoles = true;
}

/**
 * Recalculates the complete mapping from robot to role.
 */
void StaticRoleAssignment::calculateRoles()
{
    // clear current map
    _robotRoleMapping.clear();

    // get data for "calculations"
    const PlanRepository::Accessor<Role>& roles = _ae->getPlanRepository().getRoles();

    // assign a role for each robot if you have match
    for (const Agent* agent : _ae->getTeamManager().getActiveAgents()) {
        const RobotProperties& prop = agent->getProperties();
        bool roleIsAssigned = false;

        for (const Role* role : roles) {
            // make entry in the map if the roles match
            if (role->getName() == prop.getDefaultRole()) {
                ALICA_DEBUG_MSG("Static RA: Setting Role " << role->getName() << " for robot ID " << agent->getId());
                _robotRoleMapping.emplace(agent->getId(), role);

                // set own role, if its me
                if (agent->getId() == _ae->getTeamManager().getLocalAgentID() && _ownRole != role) {
                    _ownRole = role;
                    // probably nothing is reacting on this message, but anyway we send it
                    // TODO: fix this take context
                    RoleSwitch rs;
                    rs.roleID = role->getId();
                    _ae->getCommunicator().sendRoleSwitch(rs);
                }
                roleIsAssigned = true;
                break;
            }
        }

        if (!roleIsAssigned) {
            AlicaEngine::abort("RA: Could not set a role (Default: " + prop.getDefaultRole() + ") for robot: ", agent->getId());
        }
    }
}

} /* namespace alica */
