#pragma once

#include "supplementary/AgentID.h"

#include <vector>
#include <tuple>

namespace alica {
using std::get;
using std::tuple;
using std::vector;

typedef tuple<long, vector<const supplementary::AgentID*>> stdEntryPointRobot;
struct EntryPointRobots {
    EntryPointRobots()
            : entrypoint(0) {}

    long entrypoint;
    vector<const supplementary::AgentID*> robots;

    EntryPointRobots(stdEntryPointRobot& s) {
        entrypoint = get<0>(s);
        robots = get<1>(s);
    }

    stdEntryPointRobot toStandard() {
        return move(make_tuple(entrypoint, robots));
    }
};
}  // namespace alica
