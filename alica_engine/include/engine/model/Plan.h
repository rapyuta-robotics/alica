/*
 * Plan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLAN_H_
#define PLAN_H_

#include <stddef.h>
#include <string>
#include <list>
#include <map>

#include "AbstractPlan.h"

using namespace std;
namespace alica {

class EntryPoint;
class FailureState;
class SuccessState;
class PostCondition;
class State;
class SyncTransition;
class Transition;

/**
 * An ALICA plan
 */
class Plan : public AbstractPlan {
public:
    Plan(long id = 0);
    virtual ~Plan();

    virtual string toString() const;
    EntryPoint* getEntryPointTaskID(long taskID);

    const virtual string& getFileName() const;
    map<long, EntryPoint*>& getEntryPoints();
    void setEntryPoints(const map<long, EntryPoint*>& entryPoints);
    list<FailureState*>& getFailureStates();
    void setFailureStates(const list<FailureState*>& failurePoints);
    int getMaxCardinality();
    void setMaxCardinality(int maxCardinality = 0);
    int getMinCardinality();
    void setMinCardinality(int minCardinality = 0);
    PostCondition* getPostCondition();
    void setPostCondition(PostCondition* postCondition);
    list<State*>& getStates();
    void setStates(const list<State*>& states);
    list<SuccessState*>& getSuccessStates();
    void setSuccessStates(const list<SuccessState*>& succesPoints);
    list<SyncTransition*>& getSyncTransitions();
    void setSyncTransitions(const list<SyncTransition*>& syncTransitions);
    list<Transition*>& getTransitions();
    void setTransitions(const list<Transition*>& transitions);
    string getDestinationPath();
    void setDestinationPath(string destinationPath);

protected:
    int minCardinality;
    int maxCardinality;
    map<long, EntryPoint*> entryPoints;
    list<State*> states;
    list<FailureState*> failureStates;
    list<SuccessState*> successStates;
    list<SyncTransition*> syncTransitions;
    list<Transition*> transitions;
    PostCondition* postCondition;
    string destinationPath;
};

}  // namespace alica

#endif /* PLAN_H_ */
