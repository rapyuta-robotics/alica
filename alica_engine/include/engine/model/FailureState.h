/*
 * FailurePoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef FAILURESTATE_H_
#define FAILURESTATE_H_

#include <sstream>
#include <string>

#include "TerminalState.h"

namespace alica
{

/**
 * A terminal failure state in a plan. Indicates unsuccesful termination.
 */
class FailureState : public TerminalState
{
public:
    FailureState();
    virtual ~FailureState();
    std::string toString() const override;
};

} // namespace alica

#endif /* FAILURESTATE_H_ */
