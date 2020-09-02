#pragma once

#include "MockUpBehaviour.h"

#include <cstdint>

namespace alica::test
{
template <uint32_t iterationsBeforeFail>
class FailBehaviour : public MockUpBehaviour
{
public:
    FailBehaviour();

    void run(void* msg) override;

private:
    uint32_t _iterationsBeforeFail;
};

template <uint32_t iterationsBeforeFail>
FailBehaviour<iterationsBeforeFail>::FailBehaviour()
        : MockUpBehaviour("FailBehaviour")
        , _iterationsBeforeFail(iterationsBeforeFail)
{
}

template <uint32_t iterationsBeforeFail>
void FailBehaviour<iterationsBeforeFail>::run(void* msg)
{
    if (iterationsCounter() >= _iterationsBeforeFail && !this->isFailure()) {
        setFailure();
    }
    incIterationsCounter();
}
} // namespace alica::test