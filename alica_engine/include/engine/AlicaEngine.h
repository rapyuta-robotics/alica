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

    AlicaEngine(AlicaContext& ctx, const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine, const AgentID* agentID = nullptr);
    ~AlicaEngine();

    // State modifiers:
    bool init(AlicaCreators& creatorCtx);
    void start();
    void terminate();
    void stepNotify();

    // Parameter Access:
    bool getStepEngine() const;
    bool maySendMessages() const { return _maySendMessages; }

    // Module Access:
    const AuthorityManager& getAuth() const { return _auth; }
    AuthorityManager& editAuth() { return _auth; }

    const BehaviourPool& getBehaviourPool() const { return _behaviourPool; }
    BehaviourPool& editBehaviourPool() { return _behaviourPool; }

    const Logger& getLog() const { return _log; }
    Logger& editLog() { return _log; }

    const PlanBase& getPlanBase() const { return _planBase; }
    PlanBase& editPlanBase() { return _planBase; }

    const PlanParser& getPlanParser() const { return _planParser; }
    PlanParser& editPlanParser() { return _planParser; }

    const PlanRepository& getPlanRepository() const { return _planRepository; }
    PlanRepository& editPlanRepository() { return _planRepository; }

    const VariableSyncModule& getResultStore() const { return *_variableSyncModule; }
    VariableSyncModule& editResultStore() { return *_variableSyncModule; }

    const IRoleAssignment& getRoleAssignment() const { return *_roleAssignment; }
    IRoleAssignment& editRoleAssignment() { return *_roleAssignment; }

    const SyncModule& getSyncModul() const { return _syncModul; }
    SyncModule& editSyncModul() { return _syncModul; }

    const TeamManager& getTeamManager() const { return _teamManager; }
    TeamManager& editTeamManager() { return _teamManager; }

    const TeamObserver& getTeamObserver() const { return _teamObserver; }
    TeamObserver& editTeamObserver() { return _teamObserver; }

    const BlackBoard& getBlackBoard() const { return _blackboard; }
    BlackBoard& editBlackBoard() { return _blackboard; }

    // Data Access:
    const RoleSet* getRoleSet() const { return _roleSet; }

    // internals
    void setStepCalled(bool stepCalled);
    bool getStepCalled() const;
    void iterationComplete();
    int getVersion() const;

    // AlicaContext forward interface
    const IAlicaCommunication& getCommunicator() const;
    const AlicaClock& getAlicaClock() const;
    std::string getRobotName() const;
    template <class SolverType>
    SolverType& getSolver() const;
    template <class SolverType>
    bool existSolver() const;

    // AgentIDManager forwarded interface:
    AgentIDConstPtr getIdFromBytes(const std::vector<uint8_t>& vectorID);
    template <class Prototype>
    AgentIDConstPtr getId(Prototype& idPrototype);
    AgentIDConstPtr generateId(std::size_t size) { return AgentIDConstPtr(_agentIDManager.generateID(size)); }

private:
    void setStepEngine(bool stepEngine);
    // WARNING: Initialization order dependencies, do not change the declaration order of members.
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
    std::unique_ptr<AlicaClock> _alicaClock;
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
