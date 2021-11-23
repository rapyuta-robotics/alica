#pragma once

#include <list>
#include <sstream>
#include <string>

#include "AlicaElement.h"
#include "engine/Types.h"

#include <alica_solver_interface/Interval.h>
namespace alica
{

class Plan;
class State;
class Task;
class ModelManager;
class EntryPointFactory;

/**
 * An EntryPoint is used to identify the initial state of a task within a plan.
 * It also holds cardinalities and any information specific to this (task,plan) tuple.
 */
class EntryPoint : public AlicaElement
{
public:
    EntryPoint();
    EntryPoint(int64_t id, const Plan* p, const Task* t, const State* s);
    EntryPoint(const EntryPoint&) = default;
    EntryPoint(const EntryPoint&, int64_t dynamicId);
    virtual ~EntryPoint();

    /**
     * A value encoding the do-nothing task used in loosely coupled task allocation.
     */
    constexpr static int64_t IDLEID = -1; // For Idle EntryPoint...
    const static std::string IDLENAME;

    std::string toString(std::string indent = "") const override;
    static bool compareTo(const EntryPoint* ep1, const EntryPoint* ep2);

    const Task* getTask() const { return _task; }

    const Plan* getPlan() const { return _plan; }
    const State* getState() const { return _state; }

    int getMaxCardinality() const { return _cardinality.getMax(); }
    int getMinCardinality() const { return _cardinality.getMin(); }

    Interval<int> getCardinality() const { return _cardinality; }

    bool isSuccessRequired() const { return _successRequired; }

    const StateGrp& getReachableStates() const { return _reachableStates; }
    bool isStateReachable(const State* s) const;
    int getIndex() const { return _index; }

    bool isDynamic() const { return _dynamic; };
    int64_t getDynamicId() const { return _dynamicId; };
    void setDynamicId(int64_t id) { _dynamicId = id; };

private:
    friend EntryPointFactory;
    friend ModelManager;
    void computeReachabilitySet();
    void setTask(Task* task);
    void setPlan(Plan* plan);
    void setState(State* state);
    void setSuccessRequired(bool successRequired);
    void setDynamic(bool dynamic) { _dynamic = dynamic; };

    /**
     * The initial state of this entrypoint's task.
     */
    const State* _state;
    /**
     * The task of this entrypoint.
     */
    const Task* _task;
    /**
     * The plan to which this entrypoint belongs.
     */
    const Plan* _plan;
    // TODO: EntryPoint should not depend on Interval class from external solver interface library
    // Suggestion: move the Interval class, or make EntryPoint using separate minCard, maxCard
    Interval<int> _cardinality;
    /**
     * whether or not a success of this task is required for Plan to be successful. Otherwise, this task is optional.
     */
    bool _successRequired;
    /**
     * whether the entry point is dynamic i.e. if the app can instantiate a entry point at runtime with a app specific dynamic id
     */
    bool _dynamic;
    /**
     * The id specified by the app for an instance of this entry point at runtime
     */
    int64_t _dynamicId;
    /**
     * The unique index of this entrypoint in a plan's EntryPointGrp
     */
    int _index;
    /**
     * The set of states reachable from the initial state.
     */
    StateGrp _reachableStates;
};

} // namespace alica
