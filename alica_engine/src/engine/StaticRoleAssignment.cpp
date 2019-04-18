//#define STATIC_RA_DEBUG

#include "engine/StaticRoleAssignment.h"
#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/collections/RobotProperties.h"
#include "engine/containers/RoleSwitch.h"
#include "engine/teammanager/TeamManager.h"

#include <essentials/AgentIDConstPtr.h>

#include <alica_common_config/debug_output.h>

namespace alica
{

StaticRoleAssignment::StaticRoleAssignment(AlicaEngine* ae)
        : IRoleAssignment()
        , ae(ae)
        , updateRoles(false)
{
}

/**
 * Initially calculates the robot-role mapping once.
 */
void StaticRoleAssignment::init()
{
    this->calculateRoles();
}

/**
 * Triggers the recalculation of the robot-role mapping, if the updateRoles flag is set to true.
 */
void StaticRoleAssignment::tick()
{
    if (this->updateRoles) {
        this->updateRoles = false;
        this->calculateRoles();
    }
}

/**
 * Sets the updateRoles flag to true, in order to recalculate the robot-role mapping on the next tick.
 */
void StaticRoleAssignment::update()
{
    this->updateRoles = true;
}

/**
 * Recalculates the complete mapping from robot to role.
 */
void StaticRoleAssignment::calculateRoles()
{
    // clear current map
    this->robotRoleMapping.clear();

    // get data for "calculations"
    const PlanRepository::Accessor<Role>& roles = ae->getPlanRepository()->getRoles();

    // assign a role for each robot if you have match
    for (const Agent* agent : ae->getTeamManager()->getActiveAgents()) {
        const RobotProperties& prop = agent->getProperties();
        bool roleIsAssigned = false;

        for (const Role* role : roles) {
            // make entry in the map if the roles match
            if (role->getName() == prop.getDefaultRole()) {
                ALICA_DEBUG_MSG("Static RA: Setting Role " << role->getName() << " for robot ID " << agent->getId());
                this->robotRoleMapping.emplace(agent->getId(), role);

                // set own role, if its me
                if (agent->getId() == this->ae->getTeamManager()->getLocalAgentID() && this->ownRole != role) {
                    this->ownRole = role;
                    // probably nothing is reacting on this message, but anyway we send it
                    if (this->communication != nullptr) {
                        RoleSwitch rs;
                        rs.roleID = role->getId();
                        this->communication->sendRoleSwitch(rs);
                    }
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
