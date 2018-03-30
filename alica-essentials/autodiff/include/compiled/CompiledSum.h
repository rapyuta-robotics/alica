/*
 * CompiledSum.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef COMPILEDSUM_H_
#define COMPILEDSUM_H_

#include "TapeElement.h"

#include <memory>
#include <vector>

using namespace std;

namespace autodiff {
namespace compiled {
class CompiledSum : public TapeElement {
public:
    vector<int> _terms;

    void accept(shared_ptr<ITapeVisitor> visitor);
};
} /* namespace compiled */
} /* namespace autodiff */

#endif /* COMPILEDSUM_H_ */
