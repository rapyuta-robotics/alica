#pragma once

#include <memory>
#include "engine/IAlicaWorldModel.h"

namespace alica
{
    class AlicaEngine;
    class BasicPlan;
    class Configuration;

    class IPlanCreator
    {
    public:
        virtual ~IPlanCreator() {}
        virtual std::unique_ptr<BasicPlan> createPlan(int64_t planId, IAlicaWorldModel* wm) = 0;
    };

} /* namespace alica */
