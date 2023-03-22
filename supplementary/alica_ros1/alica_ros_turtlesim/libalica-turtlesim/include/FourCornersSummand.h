#pragma once

#include <engine/USummand.h>
#include <engine/logging/Logging.h>
#include <string>
#include <vector>

namespace turtlesim
{

class FourCornersSummand : public alica::USummand
{
public:
    FourCornersSummand(double weight);
    virtual ~FourCornersSummand();
    alica::UtilityInterval eval(alica::IAssignment ass, const alica::Assignment* oldAss, const alica::Blackboard* globalBlackboard) const override;
};

} /* namespace turtlesim */
