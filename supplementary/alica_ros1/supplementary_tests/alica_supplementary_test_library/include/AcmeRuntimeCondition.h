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
    AcmeRuntimeCondition();

    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
    // Factory method
    static std::shared_ptr<AcmeRuntimeCondition> create()
    {
        std::cerr << "Debug:"
                  << "AcmeRuntimeCondition created static" << std::endl;

        return std::make_shared<AcmeRuntimeCondition>();
    }
    ~AcmeRuntimeCondition() override{};
};
BOOST_DLL_ALIAS(alica::AcmeRuntimeCondition::create, AcmeRuntimeCondition)
} // namespace alica
