/*
 * Behaviour.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef BEHAVIOUR_H_
#define BEHAVIOUR_H_

using namespace std;

#include <list>
#include <string>

#include "AlicaElement.h"

namespace alica {

class BehaviourConfiguration;
class BasicBehaviour;

/**
 * Represents a Behaviour within the plan tree
 */
class Behaviour : public AlicaElement {
public:
    Behaviour();
    Behaviour(string name);
    virtual ~Behaviour();

    string toString();

    list<BehaviourConfiguration*>& getConfigurations();
    void setConfigurations(const list<BehaviourConfiguration*>& configurations);
    const string& getFileName() const;
    void setFileName(const string& fileName);
    BasicBehaviour* getImplementation();
    void setImplementation(BasicBehaviour* implementation);

private:
    /**
     * The set of static configurations of this Behaviour
     */
    list<BehaviourConfiguration*> configurations;
    /**
     * The actual implementation of this behaviour, a subclass of BasicBehaviour
     */
    BasicBehaviour* implementation;
    string fileName;
};

}  // namespace alica

#endif /* BEHAVIOUR_H_ */
