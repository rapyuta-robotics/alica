#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

#include <memory>
#include <string>

namespace alica
{
class AbstractPlan;
class ModelFactory;
class TransitionConditionFactory;
class BlackboardBlueprint;

class TransitionCondition : public AlicaElement
{
public:
    TransitionCondition() = default;
    virtual ~TransitionCondition() = default;
    const BlackboardBlueprint* getBlackboardBlueprint() const { return _blackboardBlueprint.get(); }
private:
    // TODO: Check if friends are necessary
    friend ModelFactory;
    friend TransitionConditionFactory;

    std::unique_ptr<BlackboardBlueprint> _blackboardBlueprint;
};
} // namespace alica
