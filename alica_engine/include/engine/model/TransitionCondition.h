#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

#include <memory>
#include <string>

namespace alica
{
class AbstractPlan;
class ModelFactory;
class ExpressionHandler;

class TransitionConditon : public AlicaElement
{
public:
    Condition();
    virtual ~Condition();
    const BlackboardBlueprint* getBlackboardBlueprint() const { return _blackboardBlueprint.get(); }
private:
    // TODO: Check if friends are necessary
    friend ModelFactory;
    friend ExpressionHandler;

    std::unique_ptr<BlackboardBlueprint> _blackboardBlueprint;
};
} // namespace alica
