#pragma once

#include <memory>

namespace alica
{
    class AlicaEngine;
    class BasicPlan;
    class Configuration;

    class IPlanCreator
    {
    public:
        virtual ~IPlanCreator() {}
        virtual std::unique_ptr<BasicPlan> createPlan(int64_t planId) = 0;
    };

} /* namespace alica */
