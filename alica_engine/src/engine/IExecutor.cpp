#include "engine/IExecutor.h"

namespace alica
{

void IExecutor::attachRunCb(std::function<void()>&& runCb)
{
    _run_cb = std::move(runCb);
}

} // namespace alica
