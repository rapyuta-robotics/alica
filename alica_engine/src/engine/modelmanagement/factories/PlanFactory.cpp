#include "engine/modelmanagement/factories/PlanFactory.h"

#include "engine/model/EntryPoint.h"
#include "engine/model/PreCondition.h"
#include "engine/model/State.h"
#include "engine/model/TerminalState.h"
#include "engine/model/Transition.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/AbstractPlanFactory.h"
#include "engine/modelmanagement/factories/EntryPointFactory.h"
#include "engine/modelmanagement/factories/Factory.h"
#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/modelmanagement/factories/RuntimeConditionFactory.h"
#include "engine/modelmanagement/factories/StateFactory.h"
#include "engine/modelmanagement/factories/SynchronisationFactory.h"
#include "engine/modelmanagement/factories/TerminalStateFactory.h"
#include "engine/modelmanagement/factories/TransitionFactory.h"

#include "engine/model/Plan.h"

#include <climits>
#include <vector>

namespace alica
{
Plan* PlanFactory::create(AlicaEngine* ae, const YAML::Node& node)
{
    Plan* plan = new Plan(ae, Factory::getValue<int64_t>(node, alica::Strings::id));
    Factory::setAttributes(node, plan);
    Factory::storeElement(plan, alica::Strings::plan);
    AbstractPlanFactory::setVariables(node, plan);

    if (Factory::isValid(node[alica::Strings::masterPlan])) {
        plan->_masterPlan = node[alica::Strings::masterPlan].as<bool>();
    }
    if (Factory::isValid(node[alica::Strings::frequency])) {
        plan->_frequency = node[alica::Strings::frequency].as<int>();
    }
    if (Factory::isValid(node[alica::Strings::utilityThreshold])) {
        plan->_utilityThreshold = node[alica::Strings::utilityThreshold].as<double>();
    }
    if (Factory::isValid(node[alica::Strings::entryPoints])) {
        std::vector<EntryPoint*> entryPoints = EntryPointFactory::create(node[alica::Strings::entryPoints]);
        // add to plan and summarize min/max cardinality
        int minCard = 0;
        int maxCard = 0;
        plan->_entryPoints.reserve(entryPoints.size());
        for (int i = 0; i < static_cast<int>(entryPoints.size()); ++i) {
            plan->_entryPoints.push_back(entryPoints[i]);
            minCard += entryPoints[i]->getCardinality().getMin();
            // avoid overflow for maxCard
            long tmpMax = maxCard;
            if (tmpMax + entryPoints[i]->getCardinality().getMax() > INT32_MAX) {
                maxCard = INT32_MAX;
            } else {
                maxCard += entryPoints[i]->getCardinality().getMax();
            }
        }
        plan->setMinCardinality(minCard);
        plan->setMaxCardinality(maxCard);
    }
    if (Factory::isValid(node[alica::Strings::states])) {
        const YAML::Node& states = node[alica::Strings::states];
        for (YAML::const_iterator it = states.begin(); it != states.end(); ++it) {
            State* state = nullptr;
            std::string stateType = Factory::getValue<std::string>(*it, alica::Strings::stateType);
            if (stateType.compare(alica::Strings::terminalStateType) == 0) {
                state = TerminalStateFactory::create(*it, plan);
            } else if (stateType.compare(alica::Strings::normalStateType) == 0) {
                state = StateFactory::create(*it);
            } else {
                std::cerr << "[PlanFactory] Unknown state type encountered: '" << stateType << std::endl;
            }
            plan->_states.push_back(state);
            if (state->isFailureState()) {
                plan->_failureStates.push_back((FailureState*) state);
            }
            if (state->isSuccessState()) {
                plan->_successStates.push_back((SuccessState*) state);
            }
        }
    }
    if (Factory::isValid(node[alica::Strings::transitions])) {
        const YAML::Node& transitions = node[alica::Strings::transitions];
        for (YAML::const_iterator it = transitions.begin(); it != transitions.end(); ++it) {
            plan->_transitions.push_back(TransitionFactory::create(*it, plan));
        }
    }
    if (Factory::isValid(node[alica::Strings::preCondition])) {
        plan->_preCondition = PreConditionFactory::create(node[alica::Strings::preCondition], plan);
    }
    if (Factory::isValid(node[alica::Strings::runtimeCondition])) {
        plan->_runtimeCondition = RuntimeConditionFactory::create(node[alica::Strings::runtimeCondition], plan);
    }
    if (Factory::isValid(node[alica::Strings::synchronisations])) {
        const YAML::Node& synchronisations = node[alica::Strings::synchronisations];
        for (YAML::const_iterator it = synchronisations.begin(); it != synchronisations.end(); ++it) {
            plan->_synchronisations.push_back(SynchronisationFactory::create(*it, plan));
        }
    }

    plan->_requiresParameters = Factory::getValue<bool>(node, alica::Strings::requiresParameters, false);

    return plan;
}

void PlanFactory::attachReferences()
{
    EntryPointFactory::attachReferences();
    StateFactory::attachReferences();
    TransitionFactory::attachReferences();
    SynchronisationFactory::attachReferences();
    ConditionFactory::attachReferences();
}
} // namespace alica