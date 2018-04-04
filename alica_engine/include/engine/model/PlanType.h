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
#include "engine/Types.h"

namespace alica {
class Plan;
class Parametrisation;
class ModelFactory;

class PlanType : public AbstractPlan {
public:
    PlanType();
    virtual ~PlanType();

    const virtual string& getFileName() const;
    std::string toString() const;

    const std::list<const Parametrisation*>& getParametrisation() const {return _parametrisation;}
    const PlanSet& getPlans() const {return _plans;}
private:
    friend ModelFactory;
    void setParametrisation(const list<Parametrisation*>& parametrisation);
    void setPlans(const PlanSet& plans);

    PlanSet _plans;
    std::list<const Parametrisation*> _parametrisation;
};

}  // namespace alica

#endif /* PLANTYPE_H_ */
