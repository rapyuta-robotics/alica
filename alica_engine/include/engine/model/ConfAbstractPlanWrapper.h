#pragma once

#include "engine/blackboard/KeyMapping.h"
#include "engine/model/AlicaElement.h"
#include <memory>

namespace alica
{
class AbstractPlan;
class ConfAbstractPlanWrapperFactory;

class ConfAbstractPlanWrapper : public AlicaElement
{
public:
    ConfAbstractPlanWrapper();
    virtual ~ConfAbstractPlanWrapper();

    const AbstractPlan* getAbstractPlan() const { return _abstractPlan; }
    void setAbstractPlan(const AbstractPlan* abstractPlan) { _abstractPlan = abstractPlan; }
    const KeyMapping* getKeyMapping() const { return _keyMapping.get(); }

private:
    friend ConfAbstractPlanWrapperFactory;

    const AbstractPlan* _abstractPlan;
    std::unique_ptr<KeyMapping> _keyMapping;
};
} // namespace alica
