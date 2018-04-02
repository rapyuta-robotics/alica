/*
 * TermList.cpp
 *
 *  Created on: Dec 12, 2014
 *      Author: Philipp
 */

#include "intervals/TermList.h"

namespace alica {
namespace reasoner {
namespace intervalpropagation {

TermList::TermList() {
    first = nullptr;
    last = nullptr;
}

TermList::~TermList() {
    // TODO Auto-generated destructor stub
}

bool TermList::contains(shared_ptr<Term> t) {
    if (t->next != nullptr || t->prev != nullptr)
        return true;
    return t == first;
}

shared_ptr<Term> TermList::dequeue() {
    shared_ptr<Term> ret = first;
    if (!(ret != nullptr))
        return ret;

    first = ret->next;

    if (first != nullptr)
        first->prev = nullptr;

    ret->next = nullptr;
    ret->prev = nullptr;

    if (ret == last) {
        last = nullptr;
        first = nullptr;
    }
    return ret;
}

/*public void MoveToEnd(shared_ptr<Term> t) {
 if (t == last) return;
 if (t == first) {
 last->next = t;
 t->prev = last;
 first = t->next;
 first->prev = nullptr;
 t->next = nullptr;
 last = t;
 return;
 }
 if (first==null) {
 first = t;
 last = t;
 t->prev = nullptr;
 t->next = nullptr;
 return;
 }
 if (t->next != nullptr) {
 t->next->prev = t->prev;
 t->next = nullptr;
 }

 //remove:
 if (t->prev != nullptr) {
 t->prev->next = t->next;
 }
 //add
 t->prev = last;
 last->next = t;
 last = t;
 }*/

void TermList::enqueue(shared_ptr<Term> t) {
    if (!(first != nullptr)) {
        first = t;
        last = t;
        return;
    }
    last->next = t;
    t->prev = last;
    last = t;
}

void TermList::clear() {
    shared_ptr<Term> cur = first;
    shared_ptr<Term> next = nullptr;
    while (cur != nullptr) {
        cur->prev = nullptr;
        next = cur->next;
        cur->next = nullptr;
        cur = next;
    }
    first = nullptr;
    last = nullptr;
}

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
