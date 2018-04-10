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

    std::string toString() const;

    const ParametrisationSet& getParametrisation() const { return _parametrisation; }
    const PlanSet& getPlans() const { return _plans; }

private:
    friend ModelFactory;
    void setParametrisation(const ParametrisationSet& parametrisation);
    void setPlans(const PlanSet& plans);
    void setFileName(const std::string& filename);

    PlanSet _plans;
    ParametrisationSet _parametrisation;
    std::string _fileName;
};

}  // namespace alica

#endif /* PLANTYPE_H_ */
