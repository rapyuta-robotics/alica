#pragma once

#include "Term.h"
#include "TermPtr.h"
#include "Types.h"
#include <assert.h>
namespace autodiff
{

class TermList
{
  public:
    class TermListIter
    {
      public:
        TermListIter(TermPtr t)
            : _t(t)
        {
        }
        TermPtr operator*() const { return _t; }
        bool operator!=(const TermListIter& o) const { return _t != o._t; }
        bool operator==(const TermListIter& o) const { return _t == o._t; }
        TermListIter& operator++()
        {
            assert(_t != nullptr);
            _t = _t->_next;
            return *this;
        }

      private:
        TermPtr _t;
    };

    TermList()
        : _first(nullptr)
        , _last(nullptr)
        , _size(0)
    {
    }

    TermListIter begin() const { return TermListIter(_first); }
    TermListIter end() const { return TermListIter(nullptr); }

    bool contains(const TermPtr t) const;
    TermPtr dequeue();
    void enqueue(TermPtr t);
    void enqueueUnique(TermPtr t)
    {
        if (!contains(t)) {
            enqueue(t);
        }
    }
    void clear();
    int size() const { return _size; }
    bool empty() const { return _size == 0; }

  private:
    TermPtr _first;
    TermPtr _last;
    int _size;
};

} /* namespace autodiff */
