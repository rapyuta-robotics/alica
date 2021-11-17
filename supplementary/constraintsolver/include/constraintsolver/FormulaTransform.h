#pragma once

#include <autodiff/Types.h>

#include <list>
#include <map>
#include <memory>
#include <vector>

namespace alica
{
namespace reasoner
{
namespace cnsat
{
class Clause;
class CNSat;
class Lit;
class TermEquality;
class Var;

class FormulaTransform
{
  public:
    FormulaTransform();
    ~FormulaTransform();

    void reset();
    std::shared_ptr<std::list<std::shared_ptr<Clause>>> transformToCNF(autodiff::TermPtr formula, std::shared_ptr<CNSat> solver);
    int getAtomOccurrence() const;

  protected:
    std::map<autodiff::Term*, std::shared_ptr<Var>> atoms;
    int atomOccurrence;

    std::shared_ptr<CNSat> solver;

    std::shared_ptr<TermEquality> te;

    void doTransform(std::shared_ptr<std::list<std::shared_ptr<Clause>>>& clauses);
    void performStep(std::shared_ptr<Clause>& c, std::shared_ptr<Lit>& lit, std::shared_ptr<Clause>& newClause1, std::shared_ptr<Clause>& newClause2);
    bool tryGetVar(autodiff::TermPtr t, std::shared_ptr<Var> v);
};

} // namespace cnsat
/* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */
