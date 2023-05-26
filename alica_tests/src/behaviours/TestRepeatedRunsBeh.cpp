#include <alica_tests/behaviours/TestRepeatedRunsBeh.h>
#include <engine/RunningPlan.h>
#include <memory>

namespace alica
{
TestRepeatedRunsBeh::TestRepeatedRunsBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
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

    if (_callCounter >= 10) {
        _end = getPlanContext()->getAlicaClock().now();
        AlicaTime duration = _end - _start;
        if (_callCounter >= (duration.inMilliseconds() / (1000 / 30))) {
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
