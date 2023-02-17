
#include "engine/planselector/PartialAssignmentPool.h"

#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"
#include "engine/modelmanagement/factories/EntryPointFactory.h"

namespace alica
{

PartialAssignmentPool::PartialAssignmentPool(int initialSize)
        : _pool(initialSize)
        , _curIndex(0)
{
}

void PartialAssignmentPool::increaseSize()
{
    _pool.resize(_pool.size() * 2 + 5);
}

} // namespace alica
