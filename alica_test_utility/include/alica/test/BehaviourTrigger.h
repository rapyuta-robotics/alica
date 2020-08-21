#pragma once

#include <essentials/ITrigger.hpp>

namespace alica::test
{
class BehaviourTrigger : public essentials::ITrigger
{
public:
    void trigger();
    bool behaviourFinishedRun();
    void registerCV(std::condition_variable* cv) override;

private:
    void run(bool notifyAll) override{};
    std::condition_variable* _cv;
};
} // namespace alica
