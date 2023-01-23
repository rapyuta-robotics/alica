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
class Blackboard;
class RunningPlan;

namespace test
{
class TestContext;
}

class TransitionCondition : public AlicaElement
{
public:
    TransitionCondition(std::unique_ptr<BlackboardBlueprint> blackboardBlueprint);
    bool evaluate(const RunningPlan* rp, const Blackboard* globalBlackboard, const KeyMapping* keyMapping);
    void setEvalCallback(TransitionConditionCallback cb) { _evalCallback = cb; };

    std::string getLibraryName() const;
    void setLibraryName(const std::string& libraryname);

private:
    friend class ::alica::test::TestContext;
    std::unique_ptr<Blackboard> _blackboard;
    TransitionConditionCallback _evalCallback;

    std::string _libraryName;
};
} // namespace alica
