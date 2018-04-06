/*
 * RuntimeCondition.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef RUNTIMECONDITION_H_
#define RUNTIMECONDITION_H_

#include <string>

#include "Condition.h"

namespace alica {
class RunningPlan;

class RuntimeCondition : public Condition {
public:
    RuntimeCondition(int64_t id = 0);
    virtual ~RuntimeCondition();

    std::string toString() const;
};

}  // namespace alica

#endif /* RUNTIMECONDITION_H_ */
