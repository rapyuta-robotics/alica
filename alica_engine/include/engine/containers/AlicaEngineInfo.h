#pragma once

#include <string>
#include <tuple>
#include <vector>

namespace alica
{
typedef std::tuple<uint64_t, std::string, std::string, std::string, std::string, std::string, std::vector<uint64_t>> stdAlicaEngineInfo;
struct AlicaEngineInfo
{
    AlicaEngineInfo()
            : senderID(0)
    {
    }
    uint64_t senderID;
    std::string masterPlan;
    std::string currentPlan;
    std::string currentState;
    std::string currentRole;
    std::string currentTask;
    std::vector<uint64_t> robotIDsWithMe;

    AlicaEngineInfo(stdAlicaEngineInfo&& s)
    {
        senderID = std::get<0>(s);
        masterPlan = std::move(std::get<1>(s));
        currentPlan = std::move(std::get<2>(s));
        currentState = std::move(std::get<3>(s));
        currentRole = std::move(std::get<4>(s));
        currentTask = std::move(std::get<5>(s));
        robotIDsWithMe = std::move(std::get<6>(s));
    }

    stdAlicaEngineInfo toStandard() const { return std::make_tuple(senderID, masterPlan, currentPlan, currentState, currentRole, currentTask, robotIDsWithMe); }
};
} // namespace alica
