/*
 * PlanType.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLANTYPE_H_
#define PLANTYPE_H_

#include <string>
#include <sstream>
#include <list>

#include "AbstractPlan.h"

using namespace std;
namespace alica {
class Plan;
class Parametrisation;

class PlanType : public AbstractPlan {
public:
    PlanType();
    virtual ~PlanType();

    const virtual string& getFileName() const;
    string toString();

    list<Parametrisation*>& getParametrisation();
    void setParametrisation(const list<Parametrisation*> parametrisation);
    list<Plan*>& getPlans();
    void setPlans(const list<Plan*>& plans);

protected:
    list<Plan*> plans;
    list<Parametrisation*> parametrisation;
};

}  // namespace alica

#endif /* PLANTYPE_H_ */
