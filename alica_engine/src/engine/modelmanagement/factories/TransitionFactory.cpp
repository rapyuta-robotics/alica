#include "engine/modelmanagement/factories/TransitionFactory.h"
#include "engine/model/Transition.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/VariableBindingFactory.h"
#include "engine/modelmanagement/factories/PreConditionFactory.h"

namespace alica
{
    Transition* TransitionFactory::create(const YAML::Node& transitionNode, Plan* plan)
    {
        Transition* transition = new Transition();
        Factory::setAttributes(transitionNode, transition);
        Factory::storeElement(transition, alica::Strings::transition);
        if (transitionNode[alica::Strings::inState]) {
            Factory::transitionInStateReferences.push_back(std::pair<int64_t, int64_t>(transition->getId(),  Factory::getReferencedId(transitionNode[alica::Strings::inState])));
        }
        if (transitionNode[alica::Strings::outState]) {
            Factory::transitionOutStateReferences.push_back(std::pair<int64_t, int64_t>(transition->getId(),  Factory::getReferencedId(transitionNode[alica::Strings::outState])));
        }
        if (transitionNode[alica::Strings::synchronisation]) {
            Factory::transitionSynchReferences.push_back(std::pair<int64_t, int64_t>(transition->getId(),  Factory::getReferencedId(transitionNode[alica::Strings::synchronisation])));
        }
        if (transitionNode[alica::Strings::precondition]) {
            transition->_preCondition = PreConditionFactory::create(transitionNode[alica::Strings::precondition], plan);
        }

        return transition;
    }
} // namespace alica
