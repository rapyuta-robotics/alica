#pragma once

#include <iterator>
#include <ostream>
#include <tuple>
#include <vector>

namespace alica
{

typedef std::tuple<int64_t, std::vector<alica::AgentId>> stdEntryPointRobot;
struct EntryPointRobots
{
    EntryPointRobots()
            : entrypoint(0)
    {
    }

    int64_t entrypoint;
    std::vector<alica::AgentId> robots;

    EntryPointRobots(const stdEntryPointRobot& s)
    {
        entrypoint = std::get<0>(s);
        robots = std::get<1>(s);
    }

    stdEntryPointRobot toStandard() const { return std::make_tuple(entrypoint, robots); }
};

inline std::ostream& operator<<(std::ostream& o, const EntryPointRobots& epr)
{
    o << "EP: " << epr.entrypoint << " Robots: ";
    std::copy(epr.robots.begin(), epr.robots.end(), std::ostream_iterator<alica::AgentId>(o, ", "));
    o << std::endl;
    return o;
}
} // namespace alica
