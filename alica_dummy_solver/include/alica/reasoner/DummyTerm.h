#pragma once

#include <alica_solver_interface/SolverTerm.h>
#include <map>
#include <memory>
#include <string>

namespace alica
{
namespace reasoner
{

class DummyVariable;

class DummyTerm : public alica::SolverTerm
{
  public:
    DummyTerm();
    virtual ~DummyTerm();

    void setVariable(alica::reasoner::DummyVariable*, const std::string& value);
    const std::string& getValue(alica::reasoner::DummyVariable* dv) const;
    const std::string* tryGetValue(int64_t id) const;

  private:
    std::map<int64_t, std::string> _variableValueMap;
};

} /* namespace reasoner */
} /* namespace alica */
