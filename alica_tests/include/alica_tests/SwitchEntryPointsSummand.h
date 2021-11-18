#pragma once

#include <engine/USummand.h>

#include <string>

namespace alica
{
/**
 * Expects two EntryPoints and only allows assignments where agents completely are switched between.
 */
class SwitchEntryPointsSummand : public USummand
{
public:
    SwitchEntryPointsSummand(double weight);
    virtual ~SwitchEntryPointsSummand();
    UtilityInterval eval(IAssignment ass, const Assignment* oldAss, const IAlicaWorldModel& wm) const override;
private:
    std::string toString(IAssignment ass) const;
};

} /* namespace alica */
