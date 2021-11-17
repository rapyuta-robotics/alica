#pragma once
#include "ITermVisitor.h"
#include "TermList.h"
#include "Types.h"
#include <alloca.h>
namespace autodiff
{
class Tape;

class Tape : public ITermVisitor
{
  public:
    static constexpr int MAXARITY = 2;
    Tape();
    Tape(const Tape&) = delete;
    Tape& operator=(const Tape&) = delete;

    Tape(Tape&& o);
    Tape& operator=(Tape&& o);

    ~Tape();

    void evaluate(const double* input, double* output) const;

    template <typename InputIt, typename OutputIt>
    inline void evaluate(InputIt point_begin, InputIt point_end, OutputIt value_begin) const;

    void createFrom(TermPtr top, const std::vector<VarPtr>& vars);

    bool isSet() const { return _base != nullptr; }

    virtual int visit(Abs* a) override;
    virtual int visit(And* elem) override;
    virtual int visit(Atan2* elem) override;
    virtual int visit(Constant* elem) override;
    virtual int visit(ConstPower* elem) override;
    virtual int visit(ConstraintUtility* elem) override;
    virtual int visit(Cos* elem) override;
    virtual int visit(Exp* elem) override;
    virtual int visit(LinSigmoid* elem) override;
    virtual int visit(Log* elem) override;
    virtual int visit(LTConstraint* elem) override;
    virtual int visit(LTEConstraint* elem) override;
    virtual int visit(Max* elem) override;
    virtual int visit(Min* elem) override;
    virtual int visit(Or* elem) override;
    virtual int visit(Product* elem) override;
    virtual int visit(Reification* elem) override;
    virtual int visit(Sigmoid* elem) override;
    virtual int visit(Sin* elem) override;
    virtual int visit(Sum* elem) override;
    virtual int visit(TermPower* elem) override;
    virtual int visit(Variable* var) override;

    inline const double* getValues(int idx) const
    {
        assert(idx < _tapeLength);
        return _values + (idx * _tapeWidth);
    }

  private:
    int visitTerm(Term* t);
    void prepTerms(TermPtr top, const std::vector<VarPtr>& vars);
    double* _values;
    Parameter* _params;
    EvalFunction* _functions;
    void* _base;
    TermList _allTerms;
    int _tapeWidth;
    int _tapeLength;
};

template <typename InputIt, typename OutputIt>
inline void Tape::evaluate(InputIt point_begin, InputIt point_end, OutputIt value_begin) const
{
    const int dim = _tapeWidth - 1;
    assert(std::distance(point_begin, point_end) == dim);
    double* input = static_cast<double*>(alloca(sizeof(double) * dim));
    std::copy(point_begin, point_end, input);
    for (int i = 0; i < _tapeLength; ++i) {
        _functions[i](*this, _params + i * MAXARITY, _values + i * _tapeWidth, input, dim);
    }
    std::copy(_values + (_tapeLength - 1) * _tapeWidth, _values + _tapeLength * _tapeWidth, value_begin);
}
} // namespace autodiff