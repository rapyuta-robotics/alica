
#pragma once

#include "RecursivePropagate.h"
#include "ResetIntervals.h"

#include <autodiff/Types.h>
#include <memory>
#include <vector>

namespace alica
{
namespace reasoner
{
namespace cnsat
{
class CNSat;
class Var;
} // namespace cnsat

namespace intervalpropagation
{
class RecursivePropagate;
class ResetIntervals;

class IntervalPropagator
{
  public:
    IntervalPropagator();
    virtual ~IntervalPropagator(){};

    static int updates;
    static int visits;

    void setGlobalRanges(autodiff::TermHolder& holder, std::shared_ptr<std::vector<std::shared_ptr<std::vector<double>>>> ranges,
                         std::shared_ptr<cnsat::CNSat> solver);
    bool propagate(std::shared_ptr<std::vector<std::shared_ptr<cnsat::Var>>> decisions,
                   std::shared_ptr<std::vector<std::shared_ptr<std::vector<double>>>>& completeRanges,
                   std::shared_ptr<std::vector<std::shared_ptr<cnsat::Var>>>& offenders);
    bool prePropagate(std::shared_ptr<std::vector<std::shared_ptr<cnsat::Var>>> vars);
    bool propagate(autodiff::TermPtr term);

  private:
    ResetIntervals _ri;
    RecursivePropagate _rp;

    std::shared_ptr<std::vector<std::shared_ptr<std::vector<double>>>> globalRanges;
    std::shared_ptr<std::vector<autodiff::VarPtr>> vars;
    int dim;
    std::shared_ptr<cnsat::CNSat> solver;

    bool propagateSingle(std::shared_ptr<cnsat::Var> v, bool sign);
};

} // namespace intervalpropagation
} /* namespace reasoner */
} /* namespace alica */
