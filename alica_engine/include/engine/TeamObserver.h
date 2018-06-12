#pragma once

#include <engine/Types.h>

#include <engine/AgentIDConstPtr.h>
#include <engine/AlicaClock.h>

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

    const std::map<AgentIDConstPtr, std::unique_ptr<SimplePlanTree>>& getTeamPlanTrees() const { return _simplePlanTrees; }

    int successesInPlan(const Plan* plan);
    SuccessCollection createSuccessCollection(const Plan* plan) const;
    void updateSuccessCollection(const Plan* p, SuccessCollection& sc);

    void notifyRobotLeftPlan(const AbstractPlan* plan);
    virtual void handlePlanTreeInfo(std::shared_ptr<PlanTreeInfo> incoming);
    void close();

private:
    bool updateTeamPlanTrees();
    void cleanOwnSuccessMarks(RunningPlan* root);
    std::unique_ptr<SimplePlanTree> sptFromMessage(AgentIDConstPtr agent, const IdGrp& ids, AlicaTime time) const;

    AlicaEngine* _ae;
    AgentIDConstPtr _myId;
    const RobotEngineData* me;
    TeamManager* _tm;

    std::mutex simplePlanTreeMutex; // TODO: remove this mutex
    std::mutex _msgQueueMutex;
    mutable std::mutex successMarkMutex;

    std::map<AgentIDConstPtr, std::unique_ptr<SimplePlanTree>> _simplePlanTrees;
    std::vector<std::pair<std::shared_ptr<PlanTreeInfo>, AlicaTime>> _msgQueue;
};

} /* namespace alica */
