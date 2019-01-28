#pragma once

#include "engine/AgentIDConstPtr.h"
#include "engine/AlicaContext.h"
#include "engine/BehaviourPool.h"
#include "engine/Logger.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/blackboard/BlackBoard.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/expressionhandler/ExpressionHandler.h"
#include "engine/parser/PlanParser.h"
#include "engine/syncmodule/SyncModule.h"
#include "engine/teammanager/TeamManager.h"

#include <essentials/AgentIDManager.h>
#include <string>

namespace alica
{
struct AlicaCreators;
class Plan;
class RoleSet;
class VariableSyncModule;
class IRoleAssignment;

class AlicaEngine
{
public:
    static void abort(const std::string& msg);
    template <typename T>
    static void abort(const std::string&, const T& tail);

    AlicaEngine(AlicaContext& ctx, const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine);
    ~AlicaEngine();

    // State modifiers:
    bool init(AlicaCreators& creatorCtx);
    void start();
    void terminate();
    void stepNotify();

    // Parameter Access:
    bool getStepEngine() const;
    bool maySendMessages() const { return _maySendMessages; }

    // TODO: fix this, make these functions return references (and const ref)
    // Module Access:
    AuthorityManager& getAuth() { return _auth; }
    BehaviourPool& getBehaviourPool() { return _behaviourPool; }
    Logger& getLog() { return _log; }
    PlanBase& getPlanBase() { return _planBase; }
    PlanParser& getPlanParser() { return _planParser; }
    PlanRepository& getPlanRepository() { return _planRepository; }
    VariableSyncModule& getResultStore() { return *_variableSyncModule; }
    IRoleAssignment& getRoleAssignment() { return *_roleAssignment; }
    SyncModule& getSyncModul() { return _syncModul; }
    TeamManager& getTeamManager() { return _teamManager; }
    TeamObserver& getTeamObserver() { return _teamObserver; }
    AlicaClock& getAlicaClock() { return _alicaClock; }

    const BlackBoard& getBlackBoard() const { return _blackboard; }
    BlackBoard& editBlackBoard() { return _blackboard; }

    // Data Access:
    const RoleSet* getRoleSet() const { return _roleSet; }

    // internals
    void setStepCalled(bool stepCalled);
    bool getStepCalled() const;
    void iterationComplete();

    // AlicaContext forward interface
    const IAlicaCommunication& getCommunicator() const;
    std::string getRobotName() const;
    template <class SolverType>
    SolverType& getSolver() const;
    template <class SolverType>
    bool existSolver() const;

    // AgentIDManager forwarded interface:
    AgentIDConstPtr getIdFromBytes(const std::vector<uint8_t>& vectorID);
    template <class Prototype>
    AgentIDConstPtr getId(Prototype& idPrototype);

private:
    void setStepEngine(bool stepEngine);

    AlicaContext& _ctx;
    essentials::AgentIDManager _agentIDManager;
    PlanRepository _planRepository;
    PlanParser _planParser;
    const Plan* _masterPlan;
    const RoleSet* _roleSet;
    TeamManager _teamManager;
    BehaviourPool _behaviourPool;
    TeamObserver _teamObserver;
    SyncModule _syncModul;
    AlicaClock _alicaClock;
    ExpressionHandler _expressionHandler;
    AuthorityManager _auth;
    Logger _log;
    std::unique_ptr<IRoleAssignment> _roleAssignment;
    PlanBase _planBase;
    // TODO: fix this, VariableSyncModule has circular dependency with engine header
    // VariableSyncModule _variableSyncModule;
    std::unique_ptr<VariableSyncModule> _variableSyncModule;

    BlackBoard _blackboard;
    /**
     * Switch the engine between normal operation and silent mode, in which no messages other than debugging information
     * are sent out.
     * This is useful for a robot on hot standby.
     */
    bool _maySendMessages;
    /**
     * Set to have the engine's main loop wait on a signal via MayStep
     */
    bool _stepEngine;

    /**
     * Indicates whether the engine should run with a static role assignment
     * that is based on default roles, or not.
     */
    bool _useStaticRoles;

    bool _stepCalled;
};

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing an int and receiving
 * a pointer to a corresponding AgentID object (in that case an
 * IntRobotID).
 */
template <class Prototype>
AgentIDConstPtr AlicaEngine::getId(Prototype& idPrototype)
{
    return AgentIDConstPtr(_agentIDManager.getID<Prototype>(idPrototype));
}

template <typename T>
void AlicaEngine::abort(const std::string& msg, const T& tail)
{
    std::stringstream ss;
    ss << msg << tail;
    AlicaEngine::abort(ss.str());
}

template <class SolverType>
SolverType& AlicaEngine::getSolver() const
{
    return _ctx.getSolver<SolverType>();
}

template <class SolverType>
bool AlicaEngine::existSolver() const
{
    return _ctx.existSolver<SolverType>();
}
} // namespace alica
