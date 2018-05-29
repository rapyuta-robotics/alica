#pragma once

namespace alica {

class SolverVariable {
public:
    static constexpr double minExpressibleValue = -10E29;
    static constexpr double maxExpressibleValue = 10E29;

    SolverVariable();
    virtual ~SolverVariable();
};
} /* namespace alica */
