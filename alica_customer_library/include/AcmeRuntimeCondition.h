#pragma once

#include "engine/BasicCondition.h"
#include "engine/RunningPlan.h"
#include <boost/dll/alias.hpp>

class IAlicaWorldModel;

namespace alica
{
class AcmeRuntimeCondition : public BasicCondition
{
public:
    AcmeRuntimeCondition()
    {
        std::cerr << "Debug:"
                  << "AcmeRuntimeCondition created" << std::endl;
    }

    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
    // Factory method
    static std::unique_ptr<AcmeRuntimeCondition> create()
    {
        std::cerr << "Debug:"
                  << "AcmeRuntimeCondition created static" << std::endl;

        return std::unique_ptr<AcmeRuntimeCondition>(new AcmeRuntimeCondition());
    }
    ~AcmeRuntimeCondition() override{};
};
BOOST_DLL_ALIAS(alica::AcmeRuntimeCondition::create, AcmeRuntimeCondition)
} // namespace alica
