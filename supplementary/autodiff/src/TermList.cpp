#include <TermList.h>

#include <Term.h>

#include <assert.h>

namespace autodiff
{

bool TermList::contains(const TermPtr t) const
{
    if (t->_next != nullptr) {
        return true;
    }
    return t == _last;
}

TermPtr TermList::dequeue()
{
    TermPtr ret = _first;
    if (ret == nullptr) {
        return nullptr;
    }
    --_size;
    _first = ret->_next;

    ret->_next = nullptr;

    if (ret == _last) {
        _last = nullptr;
        _first = nullptr;
    }
    return ret;
}

void TermList::enqueue(TermPtr t)
{
    assert(t->_next == nullptr);
    ++_size;
    if (_first == nullptr) {
        _first = t;
        _last = t;
        return;
    }
    _last->_next = t;
    _last = t;
}

void TermList::clear()
{
    TermPtr cur = _first;
    TermPtr next = nullptr;
    while (cur != nullptr) {
        next = cur->_next;
        cur->_next = nullptr;
        cur = next;
    }
    _first = nullptr;
    _last = nullptr;
    _size = 0;
}

} /* namespace autodiff */
