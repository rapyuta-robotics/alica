#pragma once

#include "engine/BasicPlan.h"
#include <boost/dll/alias.hpp>

namespace alica
{
class AcmePlan : public BasicPlan
{
public:
    AcmePlan(PlanContext& context);
    virtual ~AcmePlan(){};
    void run(void* msg) override{};

    // Factory method
    static std::unique_ptr<AcmePlan> create(PlanContext& context) { return std::unique_ptr<AcmePlan>(new AcmePlan(context)); }
};
BOOST_DLL_ALIAS(alica::AcmePlan::create, acmeplan)
}; // namespace alica
