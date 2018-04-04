/*
 * PreCondition.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PRECONDITION_H_
#define PRECONDITION_H_

#include "Condition.h"

namespace alica {

class ModelFactory;

/**
 * A precondition guards a Plan or a Transition.
 */
class PreCondition : public Condition {
public:
    PreCondition(int64_t id = 0);
    virtual ~PreCondition();

    std::string toString() const;

    bool isEnabled() const {return _enabled;}
    

private:
    friend ModelFactory;
    void setEnabled(bool enabled);
    bool _enabled;
};

}  // namespace alica

#endif /* PRECONDITION_H_ */
