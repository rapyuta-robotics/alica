/*
 * RuntimeCondition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef RUNTIMECONDITION_H_
#define RUNTIMECONDITION_H_

#include <string>
#include <sstream>

#include "Condition.h"

using namespace std;
namespace alica {
class RunningPlan;

class RuntimeCondition : public Condition {
public:
    RuntimeCondition(long id = 0);
    virtual ~RuntimeCondition();

    string toString();
};

}  // namespace alica

#endif /* RUNTIMECONDITION_H_ */
