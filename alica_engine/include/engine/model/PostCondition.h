/*
 * PostCondition.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef POSTCONDITION_H_
#define POSTCONDITION_H_

#include <string>
#include <sstream>

#include "Condition.h"

using namespace std;
namespace alica {

class PostCondition : public Condition {
public:
    PostCondition(long id = 0);
    virtual ~PostCondition();
    string toString();
};

}  // namespace alica

#endif /* POSTCONDITION_H_ */
