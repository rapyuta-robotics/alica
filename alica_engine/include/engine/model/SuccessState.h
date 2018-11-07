/*
 * SuccessState.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef SUCCESSSTATE_H_
#define SUCCESSSTATE_H_

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

#endif /* SUCCESSSTATE_H_ */
