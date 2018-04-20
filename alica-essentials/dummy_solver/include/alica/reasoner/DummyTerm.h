#pragma once

#include <engine/constraintmodul/SolverTerm.h>
#include <map>
#include <memory>
#include <string>

namespace alica {
namespace reasoner {

class DummyVariable;

class DummyTerm : public alica::SolverTerm {
public:
    DummyTerm();
    virtual ~DummyTerm();

    void setVariable(std::shared_ptr<alica::reasoner::DummyVariable>, std::string value);
    std::string getValue(std::shared_ptr<alica::reasoner::DummyVariable>);

private:
    std::map<long, std::string> variableValueMap;
};

} /* namespace reasoner */
} /* namespace alica */
