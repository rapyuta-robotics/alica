#pragma once

#include <string>

#include "TerminalState.h"

namespace alica
{
/**
 *  A terminal state, encoding the succesful termination of a task.
 */
class SuccessState : public TerminalState
{
public:
    SuccessState();
    virtual ~SuccessState();
    std::string toString() const override;
};

} // namespace alica
