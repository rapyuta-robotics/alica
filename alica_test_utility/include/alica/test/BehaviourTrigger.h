#pragma once

#include <essentials/ITrigger.hpp>

namespace alica::test
{
class BehaviourTrigger : public essentials::ITrigger
{
public:
    void run(bool notifyAll) override;
};
} // namespace alica
