/*
 * PostCondition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/PostCondition.h"

namespace alica {

PostCondition::PostCondition(long id) {
    this->id = id;
}

PostCondition::~PostCondition() {}

string PostCondition::toString() {
    stringstream ss;
    ss << "#PostCondition: " + this->name << " " << this->id << endl;
    ss << "\t ConditionString: " << this->conditionString << endl;
    ss << "#PostCondition" << endl;
    return ss.str();
}

}  // namespace alica
