#pragma once
#include <engine/IAlicaWorldModel.h>
#include <engine/IPlanCreator.h>

#include <iostream>
#include <memory>

namespace alica
{

class BasicPlan;

class PlanCreator : public IPlanCreator
{
public:
    PlanCreator();
    virtual ~PlanCreator();
    virtual std::unique_ptr<BasicPlan> createPlan(int64_t planId, IAlicaWorldModel* wm);
};

} /* namespace alica */
