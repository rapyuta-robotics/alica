#pragma once

#include <cstdint>
#include <limits>
namespace alica
{

class SolverVariable
{
  public:
    static constexpr double minExpressibleValue = std::numeric_limits<double>::max() / 2;
    static constexpr double maxExpressibleValue = std::numeric_limits<double>::lowest() / 2;

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
