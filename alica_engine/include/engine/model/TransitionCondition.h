#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

#include <functional>
#include <memory>
#include <string>

namespace alica
{
class AbstractPlan;
class ModelFactory;
class TransitionConditionFactory;
class BlackboardBlueprint;
class KeyMapping;
class IAlicaWorldModel;
class RunningPlan;

class TransitionCondition : public AlicaElement
{
public:
    TransitionCondition(std::unique_ptr<BlackboardBlueprint> blackboardBlueprint);
    bool evaluate(const RunningPlan* rp, const IAlicaWorldModel* wm, const KeyMapping* keyMapping);
    void setEvalCallback(TransitionConditionCallback cb) { _evalCallback = cb; };

private:
    std::unique_ptr<Blackboard> _blackboard;
    TransitionConditionCallback _evalCallback;
};
} // namespace alica
