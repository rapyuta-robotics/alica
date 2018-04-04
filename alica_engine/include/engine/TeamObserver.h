#pragma once
//#define TO_DEBUG
#include <supplementary/AgentID.h>

#include <unordered_set>
#include <string>
#include <iostream>
#include <sstream>
#include <mutex>
#include <thread>
#include <memory>
#include <ctime>
#include <map>
#include <list>

namespace alica {

class Logger;
class AlicaEngine;
class EntryPoint;
class State;
class SuccessCollection;
class TeamManager;
class RobotEngineData;
class PlanTreeInfo;
class RunningPlan;
class Plan;
class SimplePlanTree;
class AbstractPlan;

/**
 * The TeamObserver manages communication with the team. Thus it sends and receives PlanTreeInfo messages.
 * Specialized Modules may communicate through other means.
 */
class TeamObserver {
public:
    TeamObserver(AlicaEngine* ae);
    virtual ~TeamObserver();

    void tick(std::shared_ptr<RunningPlan> root);
    void doBroadCast(std::list<long>& msg);

    std::unique_ptr<
            std::map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>>
    getTeamPlanTrees();

    int successesInPlan(const Plan* plan);
    std::shared_ptr<SuccessCollection> getSuccessCollection(const Plan* plan);
    void updateSuccessCollection(const Plan* p, std::shared_ptr<SuccessCollection> sc);

    void notifyRobotLeftPlan(const AbstractPlan* plan);
    virtual void handlePlanTreeInfo(std::shared_ptr<PlanTreeInfo> incoming);
    void close();
    
private:
    const EntryPoint* entryPointOfState(const State* state) const;

    AlicaEngine* ae;
    const supplementary::AgentID* myId;
    const RobotEngineData* me;
    TeamManager* teamManager;

    std::mutex simplePlanTreeMutex;
    std::mutex successMark;

    std::shared_ptr<
            std::map<const supplementary::AgentID*, std::shared_ptr<SimplePlanTree>, supplementary::AgentIDComparator>>
            simplePlanTrees;

    void cleanOwnSuccessMarks(std::shared_ptr<RunningPlan> root);
    std::shared_ptr<SimplePlanTree> sptFromMessage(const supplementary::AgentID* robotId, const std::list<int64_t>& ids);
};

} /* namespace alica */
