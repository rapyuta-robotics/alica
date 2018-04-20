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

    std::string toString() const override;

    const ParametrisationGrp& getParametrisation() const { return _parametrisation; }
    const PlanGrp& getPlans() const { return _plans; }

private:
    friend ModelFactory;
    void setParametrisation(const ParametrisationGrp& parametrisation);
    void setPlans(const PlanGrp& plans);

    PlanGrp _plans;
    ParametrisationGrp _parametrisation;
};

}  // namespace alica

#endif /* PLANTYPE_H_ */
