#pragma once

#include <engine/BasicBehaviour.h>

namespace alica::test
{
template <uint32_t iterationsBeforeTrigger>
class SuccessOrFailBehaviour : public alica::BasicBehaviour
{
public:
    SuccessOrFailBehaviour<iterationsBeforeTrigger>(bool triggerSuccess, const std::string& nameOfMockedBehaviour);
    void run(void* msg) override;

protected:
    uint32_t iterationsCounter() const;
    void incIterationsCounter();

private:
    bool _triggerSuccess;
    uint32_t _iterationsBeforeTrigger;
    uint32_t _iterationsCounter;
};

template <uint32_t iterationsBeforeTrigger>
SuccessOrFailBehaviour<iterationsBeforeTrigger>::SuccessOrFailBehaviour(bool triggerSuccess, const std::string& nameOfMockedBehaviour)
        : BasicBehaviour(nameOfMockedBehaviour)
        , _iterationsCounter(0)
        , _iterationsBeforeTrigger(iterationsBeforeTrigger)
        , _triggerSuccess(triggerSuccess)
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
