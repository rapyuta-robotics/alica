
#include "engine/planselector/PartialAssignmentPool.h"

#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"
#include "engine/modelmanagement/factories/EntryPointFactory.h"

namespace alica
{

PartialAssignmentPool::PartialAssignmentPool(int initialSize)
        : _pool(initialSize)
        , _curIndex(0)
        , _idleEP(EntryPointFactory::generateIdleEntryPoint())
        , _idleTask(_idleEP->getTask())
{
}

PartialAssignmentPool::~PartialAssignmentPool()
{
    delete _idleEP;
    delete _idleTask;
}

void PartialAssignmentPool::increaseSize()
{
    _pool.resize(_pool.size() * 2 + 5);
}

} // namespace alica
