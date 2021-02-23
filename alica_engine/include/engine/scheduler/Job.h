#pragma once

#include "engine/AlicaClock.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <vector>
#include <memory>
#include <functional>

namespace alica {
namespace scheduler
{
struct Job
{
    Job(std::function<void()> cb, std::vector<std::weak_ptr<Job>> prerequisites)
        : cb(cb)
        , prerequisites(prerequisites)
        , cancelled(false)
        , isRepeated(false)
        , inExecution(false)
    {

    }

    std::function<void()> cb;
    AlicaTime scheduledTime;
    bool cancelled;
    bool isRepeated;
    bool inExecution;
    AlicaTime repeatInterval;
    std::vector<std::weak_ptr<Job>> prerequisites;

    bool operator==(const Job& r)
    {
        return scheduledTime == r.scheduledTime;
    }
//
//    bool operator<(const Job& r)
//    {
//        return std::find(prerequisites.begin(), prerequisites.end(), r) != prerequisites.end();
//    }

};
} //namespace scheduler
} //namespace alica
