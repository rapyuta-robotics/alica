/*
 * CompiledAnd.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDAND_H_
#define COMPILEDAND_H_

#include "TapeElement.h"

namespace autodiff {
namespace compiled {

class CompiledAnd : public TapeElement {
public:
    int _left;
    int _right;

    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDAND_H_ */
