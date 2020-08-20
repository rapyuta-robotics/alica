#pragma once

#include <essentials/ITrigger.h>

namespace alica::test
{
class BehaviourTrigger : public essentials::ITrigger
{
public:
    void trigger();
    bool behaviourFinishedRun();

private:
    void run(bool notifyAll) override{};
};
} // namespace alica
