#pragma once

#include <cstdint>
#include <limits>
namespace alica
{

class SolverVariable
{
public:
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
