/*
 * CompiledOr.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDOR_H_
#define COMPILEDOR_H_

#include "TapeElement.h"

namespace autodiff {
namespace compiled {

class CompiledOr : public TapeElement {
public:
    int _left;
    int _right;

    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDOR_H_ */
