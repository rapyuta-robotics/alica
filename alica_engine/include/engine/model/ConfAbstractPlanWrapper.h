#pragma once

#include "engine/model/AlicaElement.h"

namespace alica
{
class AbstractPlan;
class Configuration;
class ConfAbstractPlanWrapperFactory;
class ConfAbstractPlanWrapper : public AlicaElement
{
public:
    ConfAbstractPlanWrapper();
    virtual ~ConfAbstractPlanWrapper();

    const AbstractPlan* getAbstractPlan() const { return _abstractPlan; }
    void setAbstractPlan(const AbstractPlan* abstractPlan) { _abstractPlan = abstractPlan; }
    const Configuration* getConfiguration() const { return _configuration; }

private:
    friend ConfAbstractPlanWrapperFactory;

    const AbstractPlan* _abstractPlan;
    const Configuration* _configuration;
};
} // namespace alica
