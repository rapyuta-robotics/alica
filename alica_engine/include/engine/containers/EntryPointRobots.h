#pragma once

#include "supplementary/AgentID.h"

#include <tuple>
#include <vector>

namespace alica
{

typedef std::tuple<int64_t, std::vector<const supplementary::AgentID*>> stdEntryPointRobot;
struct EntryPointRobots
{
    EntryPointRobots()
        : entrypoint(0)
    {
    }

    int64_t entrypoint;
    std::vector<const supplementary::AgentID*> robots;

    EntryPointRobots(const stdEntryPointRobot& s)
    {
        entrypoint = std::get<0>(s);
        robots = std::get<1>(s);
    }

    stdEntryPointRobot toStandard() const { return std::make_tuple(entrypoint, robots); }
};
} // namespace alica
