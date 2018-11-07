/*
 * PlanRepository.cpp
 *
 *  Created on: May 28, 2014
 *      Author: Paul Panin
 */

#include "engine/PlanRepository.h"
#include "engine/Types.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Parametrisation.h"
#include "engine/model/Plan.h"
#include "engine/model/PreCondition.h"
#include "engine/model/Quantifier.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/Variable.h"

#include <cassert>
namespace alica
{

namespace
{
bool checkVarsInCondition(const Condition* c, const Plan* p)
{
    if (c == nullptr) {
        return true;
    }
    const VariableGrp& pvars = p->getVariables();
    for (const Variable* v : c->getVariables()) {
        if (std::find(pvars.begin(), pvars.end(), v) == pvars.end()) {
            std::cerr << "Variable " << v->toString() << " used in Condition " << c->toString() << " in Plan " << p->toString()
                      << " is not properly contained in the plan." << std::endl;
            assert(false);
            return false;
        }
    }

    return true;
}
bool checkVarsInParametrisations(const Plan* p)
{
    const VariableGrp& pvars = p->getVariables();
    for (const State* s : p->getStates()) {
        for (const Parametrisation* pr : s->getParametrisation()) {
            if (std::find(pvars.begin(), pvars.end(), pr->getVar()) == pvars.end()) {
                std::cerr << "Variable " << pr->getVar()->toString() << " used in Parametrisation of state " << s->toString() << " in Plan " << p->toString()
                          << " is not properly contained in the plan." << std::endl;
                assert(false);
                return false;
            }
        }
    }
    return true;
}

bool checkVarsInPlan(const Plan* p)
{
    bool ret = checkVarsInCondition(p->getPreCondition(), p);
    ret = ret && checkVarsInCondition(p->getRuntimeCondition(), p);
    ret = ret && checkVarsInParametrisations(p);

    return ret;
}
} // namespace

PlanRepository::PlanRepository() {}
PlanRepository::~PlanRepository() {}

bool PlanRepository::verifyPlanBase() const
{
    // Every entrypoint has a task:
    for (const EntryPoint* ep : getEntryPoints()) {
        if (ep->getTask() == nullptr) {
            std::cerr << "EntryPoint " << ep->toString() << " does not have a task." << std::endl;
            assert(false);
            return false;
        }
    }
    // Every plans entryPoints are sorted:
    for (const Plan* p : getPlans()) {
        for (int i = 0; i < static_cast<int>(p->getEntryPoints().size()) - 1; ++i) {
            if (p->getEntryPoints()[i]->getId() >= p->getEntryPoints()[i + 1]->getId()) {
                std::cerr << "Wrong sorting of entrypoints in plan " << p->toString() << std::endl;
                assert(false);
                return false;
            }
        }
        checkVarsInPlan(p);
    }

    return true;
}

} // namespace alica