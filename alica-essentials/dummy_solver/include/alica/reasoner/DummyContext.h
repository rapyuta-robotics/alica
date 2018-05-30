#pragma once

#include <alica_solver_interface/SolverContext.h>
#include <memory>
#include <vector>

namespace alica
{
namespace reasoner
{
class DummyVariable;

class DummyContext : public SolverContext
{
  public:
    DummyVariable* createVariable(int64_t id);
    virtual void clear() override;
    const std::vector<std::unique_ptr<DummyVariable>>& getVariables() const { return _vars; }

  private:
    std::vector<std::unique_ptr<DummyVariable>> _vars;
};
}
}