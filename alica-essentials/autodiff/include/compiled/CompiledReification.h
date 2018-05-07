/*
 * CompiledReification.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDREIFICATION_H_
#define COMPILEDREIFICATION_H_

#include "TapeElement.h"

namespace autodiff {
namespace compiled {

class CompiledReification : public TapeElement {
public:
    double _min;
    double _max;
    int _condition;
    int _negatedCondition;

    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDREIFICATION_H_ */
