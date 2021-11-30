#pragma once

#include <engine/BasicPlan.h>
#include <engine/IAlicaWorldModel.h>
#include <string>
/*PROTECTED REGION ID(domainPlanHeaderHead) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
class DomainPlan : public BasicPlan
{
public:
    DomainPlan(IAlicaWorldModel* wm);
    virtual ~DomainPlan();
    /*PROTECTED REGION ID(domainPlanClassDecl) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
};
} /* namespace alica */
