#pragma once

#include <string>

#include "TerminalState.h"

namespace alica
{

/**
 * A terminal failure state in a plan. Indicates unsuccessful termination.
 */
class FailureState : public TerminalState
{
public:
    FailureState();
    std::string toString(std::string indent = "") const override;
};

} // namespace alica
