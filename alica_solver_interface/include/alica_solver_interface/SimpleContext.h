#pragma once

#include "SolverContext.h"
#include <memory>
#include <vector>

namespace alica
{

// An example context that holds variables only
template <class VarType>
class SimpleContext : public SolverContext
{
  public:
    SimpleContext() {}
    VarType* createVariable(int64_t id)
    {
        VarType* ret = new VarType(id);
        _vars.emplace_back(ret);
        return ret;
    }
    const std::vector<std::unique_ptr<VarType>>& getVariables() const { return _vars; }
    virtual void clear() override { _vars.clear(); }

  private:
    std::vector<std::unique_ptr<VarType>> _vars;
};
}