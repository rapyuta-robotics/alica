#include "CircleRuntimeConditionConstraint.h"

#include "world_model.hpp"
#include <autodiff/AutoDiff.h>
#include <engine/RunningPlan.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/model/Task.h>

namespace turtlesim
{
void CircleRuntimeConditionConstraint::getConstraint(std::shared_ptr<alica::ProblemDescriptor> c, std::shared_ptr<alica::RunningPlan> rp)
{
    using autodiff::TermPtr;
    autodiff::TermHolder* h = static_cast<autodiff::TermHolder*>(c->getContext());
    TermPtr constraint = h->trueConstant();

    // set utility as constant which means result meet only constraints.
    // if you set non constant, ALICA will maximize the utility.
    c->setUtility(h->constant(1));

    // iterate through agents
    const int agent_num = static_cast<int>(c->getAgentVars().size());
    for (int i = 0; i < agent_num; ++i) {
        // get variables which is define in editor->Runtime condition->Quantifiers
        autodiff::Variable* x = static_cast<autodiff::Variable*>(c->getAgentVars()[i].getVars()[0]);
        autodiff::Variable* y = static_cast<autodiff::Variable*>(c->getAgentVars()[i].getVars()[1]);

        // check agent is leader
        const alica::EntryPoint* agent_entrypoint = rp->getAssignment().getEntryPointOfAgent(c->getAgentVars()[i].getId());
        if (!agent_entrypoint) {
            // Engine doesn't guarantee consistent plan view during this function execution. A null entry
            // point means that the plan status changed between the time this function was invoked
            // and entry point was requested. Don't do anything now and ignore further constraints building.
            return;
        }

        const bool is_leader = agent_entrypoint->getTask()->getName() == "Leader";

        // set range of variables
        // 0~10 is turtlesim default area size -> center is (5,5)
        x->editRange().intersect(0, 10);
        y->editRange().intersect(0, 10);
        autodiff::TVec<2> pos(x, y);

        // place turtle between radius_min and radius_max
        constexpr float tolerance = 0.1;
        constexpr float radius = 2.5;
        constexpr float radius_min = radius - tolerance;
        constexpr float radius_max = radius + tolerance;
        const autodiff::TVec<2> center(h->constant(5.0), h->constant(5.0));

        if (is_leader) // leader goes to center
        {
            constraint &= autodiff::equals(pos, center, tolerance);
        } else // follower keep distance from center and other followers
        {
            // keep distance from center
            TermPtr dist_to_center_sqr = autodiff::distanceSqr(pos, center);
            constraint &= (dist_to_center_sqr < h->constant(radius_max * radius_max));
            constraint &= (dist_to_center_sqr > h->constant(radius_min * radius_min));

            // keep distance among followers
            for (int j = i + 1; j < agent_num; ++j) {
                // check agent is leader
                // another way to check agent is Leader
                const alica::EntryPoint* other_agent_entrypoint = rp->getAssignment().getEntryPointOfAgent(c->getAgentVars()[j].getId());
                if (!other_agent_entrypoint) {
                    // Engine doesn't guarantee consistent plan view during this function execution. A null entry
                    // point means that the plan status changed between the time this function was invoked
                    // and entry point was requested. Don't do anything now and ignore further constraints building.
                    return;
                }

                const bool is_others_leader = (other_agent_entrypoint->getId() == 4346694000146342467);
                if (!is_others_leader) {
                    autodiff::Variable* x_other = static_cast<autodiff::Variable*>(c->getAgentVars()[j].getVars()[0]);
                    autodiff::Variable* y_other = static_cast<autodiff::Variable*>(c->getAgentVars()[j].getVars()[1]);
                    const autodiff::TVec<2> pos_other(x_other, y_other);
                    TermPtr dist_to_other_sqr = autodiff::distanceSqr(pos, pos_other);

                    // calculate distance for neighbors
                    float dis_between_followers = 2.0 * radius * sin(M_PI / (float) (agent_num - 1)) - tolerance;
                    constraint &= (dist_to_other_sqr > h->constant(dis_between_followers * dis_between_followers));
                }
            }
        }
    }

    c->setConstraint(constraint);
}

std::shared_ptr<CircleRuntimeConditionConstraint> CircleRuntimeConditionConstraint::create(alica::ConstraintContext&)
{
    return std::make_shared<CircleRuntimeConditionConstraint>();
}

} // namespace turtlesim
