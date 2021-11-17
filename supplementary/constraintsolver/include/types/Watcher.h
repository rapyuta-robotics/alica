/*
 * Watcher.h
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#ifndef WATCHER_H_
#define WATCHER_H_

#include <memory>

using namespace std;

namespace alica
{
namespace reasoner
{
namespace cnsat
{
class Clause;
class Lit;

class Watcher : public enable_shared_from_this<Watcher>
{
  public:
    Watcher(shared_ptr<Lit> l, shared_ptr<Clause> parent);
    virtual ~Watcher();

    shared_ptr<Clause> clause;
    shared_ptr<Lit> lit;
};

} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */

#endif /* WATCHER_H_ */
