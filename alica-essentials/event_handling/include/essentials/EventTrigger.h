#pragma once

#include "ITrigger.hpp"

namespace essentials
{
class EventTrigger : public virtual ITrigger
{
public:
    void run(bool notifyAll) override;
};
} // namespace essentials
