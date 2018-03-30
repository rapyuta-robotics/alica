/*
 * Parametrisation.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PARAMETRISATION_H_
#define PARAMETRISATION_H_

#include <string>
#include <sstream>

#include "AlicaElement.h"

using namespace std;
namespace alica {
class Variable;
class AbstractPlan;

class Parametrisation : public AlicaElement {
public:
    Parametrisation();
    virtual ~Parametrisation();

    string ToString();

    AbstractPlan* getSubPlan();
    void setSubPlan(AbstractPlan* subPlan);
    Variable* getSubVar();
    void setSubVar(Variable* subVar);
    Variable* getVar();
    void setVar(Variable* var);

protected:
    Variable* var;
    Variable* subVar;
    AbstractPlan* subPlan;
};

}  // namespace alica

#endif /* PARAMETRISATION_H_ */
