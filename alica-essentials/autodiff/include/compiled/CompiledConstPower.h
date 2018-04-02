/*
 * CompiledConstPower.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDCONSTPOWER_H_
#define COMPILEDCONSTPOWER_H_

#include "TapeElement.h"

namespace autodiff {
namespace compiled {

class CompiledConstPower : public TapeElement {
public:
    int _base;
    double _exponent;

    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDCONSTPOWER_H_ */
