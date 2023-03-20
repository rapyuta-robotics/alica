#pragma once

#include "Condition.h"

namespace alica
{
/**
 * A precondition guards a Plan, Transition, or Behaviour
 */
class PreCondition : public Condition
{
public:
    PreCondition();

    std::string toString(std::string indent = "") const override;
    void setEnabled(bool enabled);
    bool isEnabled() const { return _enabled; }

private:
    bool _enabled;
};

} // namespace alica
