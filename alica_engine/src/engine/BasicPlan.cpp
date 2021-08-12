#include "engine/BasicPlan.h"
#include "engine/AlicaEngine.h"

namespace alica
{

BasicPlan::BasicPlan() {}

void BasicPlan::start()
{
    std::function<void()> cb = std::bind(&BasicPlan::init, this);
    _ae->editScheduler().schedule(cb);
}

void BasicPlan::stop()
{
    std::function<void()> cb = std::bind(&BasicPlan::onTermination, this);
    _ae->editScheduler().schedule(cb);
}

} // namespace alica
