#include "engine/model/Condition.h"

#include "engine/BasicCondition.h"
#include "engine/BasicConstraint.h"
#include "engine/RunningPlan.h"
#include "engine/logging/IAlicaLogger.h"
#include "engine/logging/Logging.h"
#include "engine/model/Quantifier.h"

namespace alica
{

Condition::Condition()
        : _abstractPlan(nullptr)
{
}

void Condition::getConstraint(std::shared_ptr<ProblemDescriptor> pd, const RunningPlan& rp) const
{
    // TODO: fix const cast below
    _basicConstraint->getConstraint(pd, const_cast<RunningPlan&>(rp).getSharedPointer());
}

void Condition::setConditionString(const std::string& conditionString)
{
    _conditionString = conditionString;
}

bool Condition::evaluate(const RunningPlan& rp, const Blackboard* globalBlackboard) const
{
    if (_basicCondition == nullptr) {
        Logging::logDebug("Condition") << "Condition: Missing implementation of condition: ID " << getId();
        return false;
    } else {
        bool ret = false;
        try {
            // TODO: fix const cast below
            ret = _basicCondition->evaluate(const_cast<RunningPlan&>(rp).getSharedPointer(), globalBlackboard);
        } catch (const std::exception& e) {
            Logging::logDebug("Condition") << "Condition: Exception during evaluation caught:\n" << e.what();
        }
        return ret;
    }
}

void Condition::addQuantifier(const Quantifier* quantifier)
{
    _quantifiers.push_back(quantifier);
}

void Condition::addVariable(const Variable* variable)
{
    _variables.push_back(variable);
}

void Condition::setAbstractPlan(const AbstractPlan* abstractPlan)
{
    _abstractPlan = abstractPlan;
}

void Condition::setBasicCondition(const std::shared_ptr<BasicCondition>& basicCondition)
{
    _basicCondition = basicCondition;
}

void Condition::setBasicConstraint(const std::shared_ptr<BasicConstraint>& basicConstraint)
{
    _basicConstraint = basicConstraint;
}

void Condition::setLibraryName(const std::string& name)
{
    _libraryName = name;
}

} // namespace alica
