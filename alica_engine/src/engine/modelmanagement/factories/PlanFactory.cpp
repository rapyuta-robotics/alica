#include "engine/modelmanagement/factories/PlanFactory.h"

#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/Factory.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/State.h"
#include "engine/model/Transition.h"
#include "engine/model/PreCondition.h"
#include "engine/modelmanagement/factories/EntryPointFactory.h"
#include "engine/modelmanagement/factories/StateFactory.h"
#include "engine/modelmanagement/factories/TransitionFactory.h"
#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/modelmanagement/factories/RuntimeConditionFactory.h"
#include "engine/modelmanagement/factories/VariableFactory.h"
#include "engine/modelmanagement/factories/AbstractPlanFactory.h"
#include "engine/modelmanagement/factories/SynchronisationFactory.h"

#include "engine/model/Plan.h"

#include <vector>

namespace alica
{
Plan* PlanFactory::create(const YAML::Node& node)
{
    Plan* plan = new Plan(Factory::getValue<int64_t>(node, alica::Strings::id));
    Factory::setAttributes(node, plan);
    Factory::storeElement(plan, alica::Strings::plan);
    AbstractPlanFactory::setVariables(node, plan);

    if (node[alica::Strings::masterPlan]) {
        plan->_masterPlan = node[alica::Strings::masterPlan].as<bool>();
    }
    if (node[alica::Strings::utilityThreshold]) {
        plan->_utilityThreshold = node[alica::Strings::utilityThreshold].as<double>();
    }
    if (node[alica::Strings::entryPoints]) {
        std::vector<EntryPoint*> entryPoints = EntryPointFactory::create(node[alica::Strings::entryPoints]);
        // add to plan and summarize min/max cardinality
        int minCard = 0;
        int maxCard = 0;
        plan->_entryPoints.reserve(entryPoints.size());
        for (int i = 0; i < static_cast<int>(entryPoints.size()); ++i) {
            plan->_entryPoints.push_back(entryPoints[i]);
            minCard += entryPoints[i]->getCardinality().getMin();
            maxCard += entryPoints[i]->getCardinality().getMax();
        }
        plan->setMinCardinality(minCard);
        plan->setMaxCardinality(maxCard);
    }
    if (node[alica::Strings::states]) {
        const YAML::Node& states = node[alica::Strings::states];
        for (YAML::const_iterator it = states.begin(); it != states.end(); ++it) {
            State* state = StateFactory::create(*it);
            plan->_states.push_back(state);
            if (state->isFailureState()) {
                plan->_failureStates.push_back((FailureState*) state);
            }
            if (state->isSuccessState()) {
                plan->_successStates.push_back((SuccessState*) state);
            }
        }
    }
    if (node[alica::Strings::transitions]) {
        const YAML::Node &transitions = node[alica::Strings::transitions];
        for (YAML::const_iterator it = transitions.begin(); it != transitions.end(); ++it) {
            plan->_transitions.push_back(TransitionFactory::create(*it, plan));
        }
    }
    if (node[alica::Strings::preCondition]) {
        plan->_preCondition = PreConditionFactory::create(node[alica::Strings::preCondition], plan);
    }
    if (node[alica::Strings::runtimeCondition]) {
        plan->_runtimeCondition = RuntimeConditionFactory::create(node[alica::Strings::runtimeCondition], plan);
    }
    if (node[alica::Strings::synchronisations]) {
        const YAML::Node &synchronisations = node[alica::Strings::synchronisations];
        for (YAML::const_iterator it = synchronisations.begin(); it != synchronisations.end(); ++it) {
            plan->_synchronisations.push_back(SynchronisationFactory::create(*it, plan));
        }
    }

    return plan;
}
} // namespace alica