#pragma once

#include "PartialAssignment.h"
#include <stdexcept>
#include <vector>
namespace alica
{
class EntryPoint;
class Task;

class PoolExhaustedException : public std::runtime_error
{
public:
    PoolExhaustedException(const std::string& msg)
            : std::runtime_error(msg)
    {
    }
};

class PartialAssignmentPool
{
public:
    PartialAssignmentPool(int initialSize);
    ~PartialAssignmentPool();
    PartialAssignment* getNext()
    {
        PartialAssignment* pa = &_pool[_curIndex];

        if (++_curIndex >= _pool.size()) {
            throw PoolExhaustedException(std::string("Partial Assignment Pool too small at ") + std::to_string(_pool.size()));
        }
        return pa;
    }
    void increaseSize();
    void reset() { _curIndex = 0; }

private:
    std::vector<PartialAssignment> _pool;
    unsigned int _curIndex;
    const EntryPoint* _idleEP;
    const Task* _idleTask; // Do not move above idleEp
};

} // namespace alica
