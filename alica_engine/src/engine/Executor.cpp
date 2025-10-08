#include "engine/Executor.h"
#include <cassert>

namespace alica
{

void AsyncExecutor::start()
{
    assert(_run_cb);
    _running = true;
    _thread = std::make_unique<std::thread>(&AsyncExecutor::run, this);
}

void AsyncExecutor::stop()
{
    _running = false;
    if (_thread) {
        _thread->join();
        _thread.reset();
    }
}

void AsyncExecutor::run()
{
    assert(_run_cb);
    while (_running) {
        _run_cb();
    }
}

void DrivenExecutor::run()
{
    assert(_run_cb);
    _run_cb();
}

} // namespace alica
