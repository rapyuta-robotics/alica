#pragma once

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

    std::string toString(std::string indent = "") const;

    void setDeferring(int deferring);
    int getDeferring() const { return _deferring; }
    void setEventDriven(bool eventDriven);
    bool isEventDriven() const { return _eventDriven; }
    void setFrequency(int frequency);
    int getFrequency() const { return _frequency; }
    void setPreCondition(PreCondition* condition);
    const PreCondition* getPreCondition() const { return _preCondition; }
    void setRuntimeCondition(RuntimeCondition* condition);
    const RuntimeCondition* getRuntimeCondition() const { return _runtimeCondition; }
    void setPostCondition(PostCondition* condition);
    const PostCondition* getPostCondition() const { return _postCondition; }
    void setBlackboardBlueprint(std::unique_ptr<BlackboardBlueprint> blueprint);
    const BlackboardBlueprint* getBlackboardBlueprint() const { return _blackboardBlueprint.get(); }
    void setLibraryName(const std::string& name);
    std::string getLibraryName() const { return _libraryName; }
    void setImplementationName(const std::string& name);
    std::string getImplementationName() const { return !_implementationName.empty() ? _implementationName : getName(); }

private:
    friend alica::test::TestContext;

    /**
     * Specifies whether this Behaviour is run eventDriven. If it is not event driven, a timer will call it according to
     * Frequency and Deferring.
     */
    bool _eventDriven{false};
    /**
     * The frequency with which this Behaviour is called in case it is not EventDriven.
     */
    int _frequency{1};
    /**
     * The time in ms to wait before this Behaviour is executed for the first time after entering the corresponding
     * state. Has only effect for Behaviours not running in EventDriven mode.
     */
    int _deferring{0};

    /**
     * This behaviour's runtime condition.
     */
    RuntimeCondition* _runtimeCondition{nullptr};
    /**
     * This behaviour's precondition
     */
    PreCondition* _preCondition{nullptr};
    /**
     * This behaviour's postcondition
     */
    PostCondition* _postCondition{nullptr};
    /**
     * If nullptr it will simply receive a reference to its parent's Blackboard
     * Otherwise, the mapped parameters will be copied in and out on init and termination respectively
     */
    std::unique_ptr<BlackboardBlueprint> _blackboardBlueprint;

    std::string _libraryName;
    std::string _implementationName; // Name of actual implementation file.  If empty, just use the behaviour name
};

} // namespace alica
