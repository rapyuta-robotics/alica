#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class HandleFailExplicit : public AlicaTestsPlan<HandleFailExplicit>
{
public:
    HandleFailExplicit(PlanContext& context);

protected:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::HandleFailExplicit::create, HandleFailExplicit)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, HandleFailExplicitUtilityFunction)
} /* namespace alica */
