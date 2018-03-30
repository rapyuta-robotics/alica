/*
 * SuccessState.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef SUCCESSSTATE_H_
#define SUCCESSSTATE_H_

#include <string>
#include <sstream>

#include "TerminalState.h"

using namespace std;
namespace alica {
/**
 *  A terminal state, encoding the succesful termination of a task.
 */
class SuccessState : public TerminalState {
public:
    SuccessState();
    virtual ~SuccessState();
    string toString();
};

}  // namespace alica

#endif /* SUCCESSSTATE_H_ */
