#pragma once

#include "engine/blackboard/KeyMapping.h"
#include "engine/model/AlicaElement.h"
#include <memory>

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
    const KeyMapping* getKeyMapping() const { return _keyMapping.get(); }

private:
    friend ConfAbstractPlanWrapperFactory;

    const AbstractPlan* _abstractPlan;
    const Configuration* _configuration;
    std::unique_ptr<KeyMapping> _keyMapping;
};
} // namespace alica
