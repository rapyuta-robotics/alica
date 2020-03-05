#pragma once

#include <string>

#include "TerminalState.h"

namespace alica
{
/**
 *  A terminal state, encoding the successful termination of a task.
 */
class SuccessState : public TerminalState
{
public:
    SuccessState();
    virtual ~SuccessState();
    std::string toString(std::string indent = "") const override;
};

} // namespace alica
