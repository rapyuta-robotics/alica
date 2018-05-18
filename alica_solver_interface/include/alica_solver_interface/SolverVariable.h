#pragma once

#include <cstdint>

namespace alica
{

class SolverVariable
{
  public:
    static constexpr double minExpressibleValue = -10E29;
    static constexpr double maxExpressibleValue = 10E29;

    SolverVariable(int64_t id)
        : _id(id)
    {
    }
    virtual ~SolverVariable() {}
    int64_t getId() const { return _id; }

  private:
    int64_t _id;
};
} /* namespace alica */
