#pragma once

#include <alica_solver_interface/SolverTerm.h>
#include <map>
#include <memory>
#include <string>

namespace alica
{
namespace reasoner
{

class SimpleVariable;

class SimpleTerm : public alica::SolverTerm
{
public:
    SimpleTerm();
    virtual ~SimpleTerm();

    void setVariable(alica::reasoner::SimpleVariable*, const std::string& value);
    const std::string& getValue(alica::reasoner::SimpleVariable* dv) const;
    const std::string* tryGetValue(int64_t id) const;

private:
    std::map<int64_t, std::string> _variableValueMap;
};

} /* namespace reasoner */
} /* namespace alica */
