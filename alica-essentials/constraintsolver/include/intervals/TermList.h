/*
 * TermList.h
 *
 *  Created on: Dec 12, 2014
 *      Author: Philipp
 */

#ifndef TERMLIST_H_
#define TERMLIST_H_

#include <AutoDiff.h>

#include <memory>

using namespace std;
using namespace autodiff;

namespace alica {
namespace reasoner {
namespace intervalpropagation {

class TermList {
public:
    TermList();
    virtual ~TermList();

    shared_ptr<Term> first;
    shared_ptr<Term> last;

    bool contains(shared_ptr<Term> t);
    shared_ptr<Term> dequeue();
    void enqueue(shared_ptr<Term> t);
    void clear();
};

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */

#endif /* TERMLIST_H_ */
