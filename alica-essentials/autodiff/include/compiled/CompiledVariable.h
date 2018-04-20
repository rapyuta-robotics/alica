/*
 * CompiledVariable.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef COMPILEDVARIABLE_H_
#define COMPILEDVARIABLE_H_

#include "TapeElement.h"

#include <memory>

using namespace std;

namespace autodiff {
namespace compiled {
class CompiledVariable : public TapeElement {
public:
    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDVARIABLE_H_ */
