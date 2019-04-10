#pragma once

#include "Condition.h"

namespace alica
{

class ModelFactory;
class PreConditionFactory;

/**
 * A precondition guards a Plan, Transition, or Behaviour
 */
class PreCondition : public Condition
{
public:
    PreCondition();
    virtual ~PreCondition();

    std::string toString(std::string indent = "") const override;

    bool isEnabled() const { return _enabled; }

private:
    friend ModelFactory;
    friend PreConditionFactory;
    void setEnabled(bool enabled);
    bool _enabled;
};

} // namespace alica