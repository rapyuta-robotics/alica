/*
 * CompiledConstraintUtility.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDCONSTRAINTUTILITY_H_
#define COMPILEDCONSTRAINTUTILITY_H_

#include "TapeElement.h"

namespace autodiff {
namespace compiled {

class CompiledConstraintUtility : public TapeElement {
public:
    int _constraint;
    int _utility;

    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDCONSTRAINTUTILITY_H_ */
