#pragma once

#include "ITrigger.hpp"

namespace essentials
{
class EventTrigger : public ITrigger
{
public:
    void run(bool notifyAll) override;
};
} // namespace essentials
