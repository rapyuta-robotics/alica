#pragma once

#include <engine/Types.h>

#include <engine/AgentIDConstPtr.h>

#include <map>
#include <memory>
#include <mutex>

namespace alica
{

class Logger;
class AlicaEngine;
class SuccessCollection;
class TeamManager;
class RobotEngineData;
class PlanTreeInfo;
class RunningPlan;
class SimplePlanTree;

/**
 * The TeamObserver manages communication with the team. Thus it sends and receives PlanTreeInfo messages.
 * Specialized Modules may communicate through other means.
 */
class TeamObserver
{
public:
    TeamObserver(AlicaEngine* ae);
    virtual ~TeamObserver();

    void tick(RunningPlan* root);
    void doBroadCast(const IdGrp& msg) const;

    std::unique_ptr<std::map<AgentIDConstPtr, std::shared_ptr<SimplePlanTree>>> getTeamPlanTrees();

    int successesInPlan(const Plan* plan);
    SuccessCollection createSuccessCollection(const Plan* plan) const;
    void updateSuccessCollection(const Plan* p, std::shared_ptr<SuccessCollection> sc);

    void notifyRobotLeftPlan(const AbstractPlan* plan);
    virtual void handlePlanTreeInfo(std::shared_ptr<PlanTreeInfo> incoming);
    void close();

private:
    const EntryPoint* entryPointOfState(const State* state) const;

    AlicaEngine* ae;
    AgentIDConstPtr myId;
    const RobotEngineData* me;
    TeamManager* teamManager;

    std::mutex simplePlanTreeMutex;
    mutable std::mutex successMarkMutex;

    std::shared_ptr<std::map<AgentIDConstPtr, std::shared_ptr<SimplePlanTree>>> simplePlanTrees;

    void cleanOwnSuccessMarks(RunningPlan* root);
    std::shared_ptr<SimplePlanTree> sptFromMessage(AgentIDConstPtr robotId, const IdGrp& ids) const;
};

} /* namespace alica */
