#pragma once

#include "ITrigger.h"

#include <condition_variable>
#include <mutex>
#include <vector>

namespace essentials
{
class EventTrigger : public virtual ITrigger
{
public:
    EventTrigger();
    virtual ~EventTrigger();
    void run(bool notifyAll = true);
};
} // namespace essentials
