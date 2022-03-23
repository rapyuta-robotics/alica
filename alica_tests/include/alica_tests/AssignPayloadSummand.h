#pragma once

#include <engine/USummand.h>
#include <string>
#include <vector>

namespace alica
{

class AssignPayloadSummand : public USummand
{
public:
    AssignPayloadSummand(double weight);
    virtual ~AssignPayloadSummand();
    UtilityInterval eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel* wm) const override;
    uint64_t movePayloadEpId = 2603044554417791500;
};

} /* namespace alica */
