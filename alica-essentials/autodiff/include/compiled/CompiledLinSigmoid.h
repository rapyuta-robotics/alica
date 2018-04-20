/*
 * CompiledLinSigmoid.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDLINSIGMOID_H_
#define COMPILEDLINSIGMOID_H_

#include "TapeElement.h"

namespace autodiff {
namespace compiled {

class CompiledLinSigmoid : public TapeElement {
public:
    int _arg;

    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDLINSIGMOID_H_ */
