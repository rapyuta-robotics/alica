/*
 * CompiledLog.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef COMPILEDLOG_H_
#define COMPILEDLOG_H_

#include "TapeElement.h"

namespace autodiff {
namespace compiled {

class CompiledLog : public TapeElement {
public:
    int _arg;

    void accept(shared_ptr<ITapeVisitor> visitor);
};

} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDLOG_H_ */
