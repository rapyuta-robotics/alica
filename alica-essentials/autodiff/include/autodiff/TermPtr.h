#pragma once
#include <utility>
namespace autodiff
{
class Term;
class TermHolder;
class TermPtr
{
  public:
    TermPtr()
        : _ptr(nullptr)
    {
    }
    TermPtr(std::nullptr_t)
        : TermPtr()
    {
    }

    TermPtr(Term* t)
        : _ptr(t)
    {
    }
    // Somewhat hacky trick for the moment:
    TermPtr(const Term* t)
        : _ptr(const_cast<Term*>(t))
    {
    }

    Term* get() const { return _ptr; }
    Term* operator->() const { return _ptr; }

    template <typename T>
    operator T*() const
    {
        return static_cast<T*>(_ptr);
    }

    bool operator==(const TermPtr o) const { return _ptr == o._ptr; }
    bool operator!=(const TermPtr o) const { return _ptr != o._ptr; }

  private:
    friend TermHolder;
    Term* _ptr;
};
}