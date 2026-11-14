#include <alica_tests/behaviours/TestRepeatedRunsBeh.h>
#include <engine/RunningPlan.h>
#include <engine/logging/Logging.h>
#include <memory>

namespace alica
{
TestRepeatedRunsBeh::TestRepeatedRunsBeh(BehaviourContext& context)
        : BasicBehaviour(context)
        , _callCounter(0)
        , _frequency(context.behaviourModel->getFrequency())
{
}

void TestRepeatedRunsBeh::run()
{
    if (isSuccess()) {
        return;
    }

    AlicaTime now = getPlanContext()->getAlicaClock().now();
    if (_callCounter == 0) {
        _start = now;
    } else if (now - _last > _maxGap) {
        _maxGap = now - _last;
    }
    _last = now;

    _callCounter++;

    // run for one second
    if (_callCounter >= _frequency) {
        _end = getPlanContext()->getAlicaClock().now();
        AlicaTime duration = _end - _start;
        if (std::abs(_callCounter - duration.inMilliseconds() * _frequency / 1000) < 5) {
            setSuccess();
        } else if (!isFailure()) {
            Logging::logWarn("TestRepeatedRunsBeh") << _callCounter << " run calls in " << duration.inMilliseconds() << "ms at " << _frequency
                                                    << "Hz, longest gap " << _maxGap.inMilliseconds() << "ms";
            setFailure();
        }
    }
}

std::unique_ptr<TestRepeatedRunsBeh> TestRepeatedRunsBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<TestRepeatedRunsBeh>(context);
}

} /* namespace alica */
