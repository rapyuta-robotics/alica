#pragma once

#include "IMockUpBehaviour.h"

#include <cstdint>

namespace alica::test
{
template <uint32_t iterationsBeforeFail>
class FailBehaviour : public IMockUpBehaviour
{
public:
    FailBehaviour();

    void run(void* msg) override;

private:
    uint32_t _iterationsBeforeFail;
};

template <uint32_t iterationsBeforeFail>
FailBehaviour<iterationsBeforeFail>::FailBehaviour()
        : IMockUpBehaviour("FailBehaviour")
        , _iterationsBeforeFail(iterationsBeforeFail)
{
}

template <uint32_t iterationsBeforeFail>
void FailBehaviour<iterationsBeforeFail>::run(void* msg)
{
    if (iterationsCounter() >= _iterationsBeforeFail && !this->isFailure()) {
        this->setFailure();
    }
    incIterationsCounter();
}
} // namespace alica::test