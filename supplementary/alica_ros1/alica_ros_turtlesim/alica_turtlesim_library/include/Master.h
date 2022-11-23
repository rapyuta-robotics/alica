#pragma once

#include "engine/BasicPlan.h"
#include "world_model.hpp"
#include <boost/dll/alias.hpp>

namespace alica
{

class Master : public BasicPlan
{
public:
    Master(PlanContext& context);
    virtual ~Master();
    // Factory method
    static std::unique_ptr<Master> create(PlanContext& context) { return std::make_unique<Master>(context); }

protected:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::Master::create, Master)
} /* namespace alica */
