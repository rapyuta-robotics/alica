
#include "engine/planselector/PartialAssignmentPool.h"

#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"
#include "engine/parser/ModelFactory.h"

namespace alica
{

PartialAssignmentPool::PartialAssignmentPool(int initialSize)
        : _pool(initialSize)
        , _curIndex(0)
        , _idleEP(ModelFactory::generateIdleEntryPoint())
        , _idleTask(idleEP->getTask())
{
}

PartialAssignmentPool::~PartialAssignmentPool()
{
    delete idleEP;
    delete idleTask;
}

void PartialAssignmentPool::increaseSize()
{
    _pool.resize(_pool.size() * 2 + 5);
}

} // namespace alica
