#pragma once

#include <engine/BasicBehaviour.h>

namespace alica::test
{
template <uint32_t iterationsBeforeTrigger>
class SuccessOrFailBehaviour : public alica::BasicBehaviour
{
public:
    SuccessOrFailBehaviour<iterationsBeforeTrigger>();
    void run(void* msg) override;
    void setTriggerType(bool triggerSuccess);

protected:
    uint32_t iterationsCounter() const;
    void incIterationsCounter();
    void initialiseParameters() override;

private:
    std::mutex _triggerMutex;
    bool _triggerTypeSet;
    bool _triggerSuccess;
    uint32_t _iterationsBeforeTrigger;
    uint32_t _iterationsCounter;
};

template <uint32_t iterationsBeforeTrigger>
SuccessOrFailBehaviour<iterationsBeforeTrigger>::SuccessOrFailBehaviour()
        : BasicBehaviour("SuccessOrFailBehaviour")
        , _iterationsCounter(0)
        , _iterationsBeforeTrigger(iterationsBeforeTrigger)
        , _triggerTypeSet(false)
        , _triggerSuccess(false)
{
}

template <uint32_t iterationsBeforeTrigger>
uint32_t SuccessOrFailBehaviour<iterationsBeforeTrigger>::iterationsCounter() const
{
    return _iterationsCounter;
}

template <uint32_t iterationsBeforeTrigger>
void SuccessOrFailBehaviour<iterationsBeforeTrigger>::incIterationsCounter()
{
    _iterationsCounter++;
}

template <uint32_t iterationsBeforeTrigger>
void SuccessOrFailBehaviour<iterationsBeforeTrigger>::setTriggerType(bool triggerSuccess)
{
    std::lock_guard<std::mutex> lockGuard(_triggerMutex);
    _triggerSuccess = triggerSuccess;
    _triggerTypeSet = true;
}

template <uint32_t iterationsBeforeTrigger>
void SuccessOrFailBehaviour<iterationsBeforeTrigger>::initialiseParameters()
{
    // Note: We require the BehaviourMockups to be default constructable, therefore we cannot set the _triggerSuccess flag on construction time.
    std::lock_guard<std::mutex> lockGuard(_triggerMutex);
    assert(_triggerTypeSet && "[SuccessOrFailBehavour] Please call setTriggerType(bool triggerSuccess) once before starting the behaviour the first time.");
}

template <uint32_t iterationsBeforeTrigger>
void SuccessOrFailBehaviour<iterationsBeforeTrigger>::run(void* msg)
{
    if (iterationsCounter() < _iterationsBeforeTrigger) {
        incIterationsCounter();
        return;
    }

    if (_triggerSuccess && !isSuccess()) {
        setSuccess();
    }

    if (!_triggerSuccess && !isFailure()) {
        setFailure();
    }
}

} // namespace alica::test
