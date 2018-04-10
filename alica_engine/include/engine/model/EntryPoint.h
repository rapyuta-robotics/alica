/*
 * EntryPoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ENTRYPOINT_H_
#define ENTRYPOINT_H_

#include <string>
#include <sstream>
#include <list>

#include "AlicaElement.h"
#include "engine/Types.h"
namespace alica {

class Plan;
class State;
class Task;

/**
 * An EntryPoint is used to identify the initial state of a task within a plan.
 * It also holds cardinalities and any information specific to this (task,plan) tuple.
 */
class EntryPoint : public AlicaElement {
public:
    EntryPoint();
    virtual ~EntryPoint();

    /**
     * A value encoding the do-nothing task used in loosely coupled task allocation.
     */
    constexpr static int64_t IDLEID = -1;  // For Idle EntryPoint...

    std::string toString() const;
    static bool compareTo(const EntryPoint* ep1, const EntryPoint* ep2);

    const Task* getTask() const { return _task; }

    const Plan* getPlan() const { return _plan; }
    const State* getState() const { return _state; }

    int getMaxCardinality() const { return _maxCardinality; }
    int getMinCardinality() const { return _minCardinality; }
    bool isSuccessRequired() const { return _successRequired; }

    const StateSet& getReachableStates() const { return _reachableStates; }
    bool isStateReachable(const State* s) const;

private:
    friend ModelFactory;
    void computeReachabilitySet();
    void setTask(Task* task);
    void setPlan(Plan* plan);
    void setState(State* state);
    void setMaxCardinality(int maxCardinality);
    void setMinCardinality(int minCardinality);
    void setSuccessRequired(bool successRequired);

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
    /**
     * The minimum amount of agents required to execute this entrypoint's task within Plan.
     */
    int _minCardinality;
    /**
     * The maximum amount of agents allowed to execute this entrypoint's task within Plan.
     */
    int _maxCardinality;
    /**
     * whether or not a success of this task is required for Plan to be successful. Otherwise, this task is optional.
     */
    bool _successRequired;
    /**
     * The set of states reachable from the initial state.
     */
    StateSet _reachableStates;
};

}  // namespace alica

#endif /* ENTRYPOINT_H_ */
