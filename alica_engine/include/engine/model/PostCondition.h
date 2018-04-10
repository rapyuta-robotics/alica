/*
 * PostCondition.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef POSTCONDITION_H_
#define POSTCONDITION_H_

#include <string>

#include "Condition.h"

namespace alica {

class PostCondition : public Condition {
public:
    PostCondition(int64_t id = 0);
    virtual ~PostCondition();
    std::string toString() const;
};

}  // namespace alica

#endif /* POSTCONDITION_H_ */
