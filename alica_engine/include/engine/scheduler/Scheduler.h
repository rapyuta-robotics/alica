#pragma once

#include "engine/scheduler/Job.h"

namespace alica
{
namespace scheduler
{
    class Scheduler
    {
    public:
        virtual void add(Job) = 0;
    };
}
}
