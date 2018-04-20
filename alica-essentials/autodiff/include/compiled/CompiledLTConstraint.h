/*
 * CompiledLTConstraint.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDLTCONSTRAINT_H_
#define COMPILEDLTCONSTRAINT_H_

#include "TapeElement.h"

#include <iostream>

using namespace std;

namespace autodiff {
namespace compiled {

class CompiledLTConstraint : public TapeElement {
public:
    int _left;
    int _right;
    double _steepness;

    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDLTCONSTRAINT_H_ */
