/*
 * CompiledLTEConstraint.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDLTECONSTRAINT_H_
#define COMPILEDLTECONSTRAINT_H_

#include "TapeElement.h"

#include <iostream>

using namespace std;

namespace autodiff {
namespace compiled {

class CompiledLTEConstraint : public TapeElement {
public:
    int _left;
    int _right;
    double _steepness;

    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDLTECONSTRAINT_H_ */
