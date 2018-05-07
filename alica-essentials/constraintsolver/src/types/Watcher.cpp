/*
 * Watcher.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#include "types/Watcher.h"

#include "types/Lit.h"
#include "types/Var.h"

namespace alica {
namespace reasoner {
namespace cnsat {

Watcher::Watcher(shared_ptr<Lit> l, shared_ptr<Clause> parent) {
    this->clause = parent;
    this->lit = l;
    this->lit->var->watchList->push_back(this);
}

Watcher::~Watcher() {
    // TODO Auto-generated destructor stub
}

} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */
