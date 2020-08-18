#pragma once

#include <essentials/ITrigger.h>

namespace alica
{
class AlicaTestBehaviourTrigger : public essentials::ITrigger
{
public:
    void trigger();
    bool behaviourFinishedRun();

private:
    void run(bool notifyAll) override{};
};
} // namespace alica
