#pragma once

#include <list>
#include <string>

#include "AbstractPlan.h"
#include "engine/Types.h"
#include "engine/blackboard/BlackboardBlueprint.h"

namespace alica
{
namespace test
{
class TestContext;
}

class BasicBehaviour;
class BehaviourFactory;
class PreCondition;
class RuntimeCondition;
class PostCondition;

/**
 * Represents a Behaviour within the plan tree
 */
class Behaviour : public AbstractPlan
{
public:
    Behaviour();
    virtual ~Behaviour();

    std::string toString(std::string indent = "") const;

    int getFrequency() const { return _frequency; }
    const RuntimeCondition* getRuntimeCondition() const { return _runtimeCondition; }
    const PreCondition* getPreCondition() const { return _preCondition; }
    const PostCondition* getPostCondition() const { return _postCondition; }
    const BlackboardBlueprint* getBlackboardBlueprint() const { return _blackboardBlueprint.get(); }
    std::string getLibraryName() const { return _libraryName; };
    std::string getImplementationName() const { return !_implementationName.empty() ? _implementationName : getName(); }

private:
    friend BehaviourFactory;
    friend alica::test::TestContext;

    void setFrequency(int frequency);
    /**
     * The frequency with which this Behaviour is called.
     */
    int _frequency;

    /**
     * This behaviour's runtime condition.
     */
    RuntimeCondition* _runtimeCondition;
    /**
     * This behaviour's precondition
     */
    PreCondition* _preCondition;
    /**
     * This behaviour's postcondition
     */
    PostCondition* _postCondition;
    /**
     * If nullptr it will simply receive a reference to its parent's Blackboard
     * Otherwise, the mapped parameters will be copied in and out on init and termination respectively
     */
    std::unique_ptr<BlackboardBlueprint> _blackboardBlueprint;

    std::string _libraryName;
    std::string _implementationName; // Name of actual implementation file.  If empty, just use the behaviour name
};

} // namespace alica
