/*
 * Behaviour.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef BEHAVIOUR_H_
#define BEHAVIOUR_H_

#include <list>
#include <string>

#include "AlicaElement.h"
#include "engine/Types.h"

namespace alica {

class BehaviourConfiguration;
class BasicBehaviour;
class ModelFactory;

/**
 * Represents a Behaviour within the plan tree
 */
class Behaviour : public AlicaElement {
public:
    Behaviour();
    virtual ~Behaviour();

    std::string toString() const override;

    const BehaviourConfigurationGrp& getConfigurations() const { return _configurations; }
    const std::string& getFileName() const { return _fileName; }
    BasicBehaviour* getImplementation() const { return _implementation; }

private:
    friend ModelFactory;

    void setFileName(const std::string& fileName);
    void setImplementation(BasicBehaviour* implementation);
    void setConfigurations(const BehaviourConfigurationGrp& configurations);
    /**
     * The set of static configurations of this Behaviour
     */
    BehaviourConfigurationGrp _configurations;
    /**
     * The actual implementation of this behaviour, a subclass of BasicBehaviour
     */
    BasicBehaviour* _implementation;
    std::string _fileName;
};

}  // namespace alica

#endif /* BEHAVIOUR_H_ */
