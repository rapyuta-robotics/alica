/*
 * Parametrisation.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PARAMETRISATION_H_
#define PARAMETRISATION_H_

#include <string>

#include "AlicaElement.h"

namespace alica
{
class Variable;
class AbstractPlan;

class Parametrisation : public AlicaElement
{
public:
    Parametrisation();
    virtual ~Parametrisation();

    std::string toString() const override;

    const AbstractPlan* getSubPlan() const { return _subPlan; }
    const Variable* getVar() const { return _var; }
    const Variable* getSubVar() const { return _subVar; }

protected:
    friend ModelFactory;

    void setSubPlan(const AbstractPlan* subPlan);
    void setSubVar(const Variable* subVar);
    void setVar(const Variable* var);

    const Variable* _var;
    const Variable* _subVar;
    const AbstractPlan* _subPlan;
};

} // namespace alica

#endif /* PARAMETRISATION_H_ */
