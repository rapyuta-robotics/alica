#include <alica/conditions/conditions.h>

#include <engine/IAlicaWorldModel.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <alica_ros_turtlesim/world_model.hpp>

namespace alica
{
bool conditionMove2Init748720375848597116(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1136497454350831106) ENABLED START*/
    return turtlesim::ALICATurtleWorldModel::get()->getInit();
    /*PROTECTED REGION END*/
}
bool conditionInit2Move974606107671315045(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1597434482701133956) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Success);
    /*PROTECTED REGION END*/
}
} /* namespace alica */
