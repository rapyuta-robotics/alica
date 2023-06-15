#include <alica_tests/behaviours/TestRepeatedRunsBeh.h>
#include <engine/RunningPlan.h>
#include <memory>

namespace alica
{
TestRepeatedRunsBeh::TestRepeatedRunsBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
    _frequency = context.behaviourModel->getFrequency();
}
void TestRepeatedRunsBeh::run()
{
    if (isSuccess()) {
        return;
    }

    if (_callCounter == 0) {
        _start = getPlanContext()->getAlicaClock().now();
    }

    _callCounter++;

    // run for one second
    if (_callCounter >= _frequency) {
        _end = getPlanContext()->getAlicaClock().now();
        AlicaTime duration = _end - _start;
        if (std::abs(_callCounter - duration.inMilliseconds() * _frequency / 1000) < 5) {
            setSuccess();
        } else {
            setFailure();
        }
    }
}

std::unique_ptr<TestRepeatedRunsBeh> TestRepeatedRunsBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<TestRepeatedRunsBeh>(context);
}

} /* namespace alica */
