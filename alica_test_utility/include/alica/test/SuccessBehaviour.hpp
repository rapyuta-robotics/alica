#pragma once

#include "MockUpBehaviour.h"

#include <cstdint>

namespace alica::test
{
template <uint32_t iterationsBeforeSuccess>
class SuccessBehaviour : public MockUpBehaviour
{
public:
    SuccessBehaviour();

    void run(void* msg) override;

private:
    uint32_t _iterationsBeforeSuccess;
};

template <uint32_t iterationsBeforeSuccess>
SuccessBehaviour<iterationsBeforeSuccess>::SuccessBehaviour()
        : MockUpBehaviour("SuccessBehaviour")
        , _iterationsBeforeSuccess(iterationsBeforeSuccess)
{
}

template <uint32_t iterationsBeforeSuccess>
void SuccessBehaviour<iterationsBeforeSuccess>::run(void* msg)
{
    if (iterationsCounter() >= _iterationsBeforeSuccess && !this->isSuccess()) {
        setSuccess();
    }
    incIterationsCounter();
}
} // namespace alica::test