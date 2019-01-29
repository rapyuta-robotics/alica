#pragma once

#include "engine/AgentIDConstPtr.h"
#include "engine/blackboard/BlackBoard.h"
#include "engine/constraintmodul/ISolver.h"

#include <SystemConfig.h>
#include <list>
#include <string>
#include <essentials/AgentIDManager.h>

#include <unordered_map>

namespace alica
{
class AlicaClock;
class PlanRepository;
class Plan;
class PlanParser;
class BehaviourPool;
class Logger;
class RoleSet;
class TeamObserver;
class SyncModule;
class AuthorityManager;
class PlanBase;
class ExpressionHandler;
class VariableSyncModule;
class TeamManager;

class IBehaviourCreator;
class IUtilityCreator;
class IConditionCreator;
class IConstraintCreator;

class IAlicaCommunication;

class IRoleAssignment;

class AlicaEngine
{
public:
    static void abort(const std::string& msg);
    template <typename T>
    static void abort(const std::string&, const T& tail);

    AlicaEngine(
            supplementary::AgentIDManager* idManager, const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine, bool hacked = false);
    ~AlicaEngine();

    // State modifiers:
    bool init(IBehaviourCreator* bc, IConditionCreator* cc, IUtilityCreator* uc, IConstraintCreator* crc);
    void shutdown();
    void start();
    void stepNotify();

    // Parameter Access:
    bool isTerminating() const;
    bool getStepEngine() const;
    bool maySendMessages() const { return _maySendMessages; }
    std::string getRobotName() const;

    // Module Access:
    AuthorityManager* getAuth() const { return auth; }
    BehaviourPool* getBehaviourPool() const { return behaviourPool; }
    const IAlicaCommunication* getCommunicator() const { return communicator; }
    Logger* getLog() const { return log; }
    PlanBase* getPlanBase() const { return planBase; }
    PlanParser* getPlanParser() const { return planParser; }
    PlanRepository* getPlanRepository() const { return planRepository; }
    VariableSyncModule* getResultStore() const { return variableSyncModule; }
    IRoleAssignment* getRoleAssignment() const { return roleAssignment; }
    SyncModule* getSyncModul() const { return syncModul; }
    TeamManager* getTeamManager() const { return _teamManager; }
    TeamObserver* getTeamObserver() const { return teamObserver; }
    AlicaClock* getAlicaClock() const { return alicaClock; }

    const BlackBoard& getBlackBoard() const { return _blackboard; }
    BlackBoard& editBlackBoard() { return _blackboard; }
    // Solver Access:
    template <class SolverType>
    void addSolver(SolverType* solver);
    template <class SolverType>
    SolverType* getSolver() const;

    // Data Access:

    const RoleSet* getRoleSet() const { return roleSet; }

    // Setters:
    void setMaySendMessages(bool maySendMessages);

    // Module Setters TODO: remove and replace with a better configuration system
    void setLog(Logger* log);
    void setTeamObserver(TeamObserver* teamObserver);
    void setSyncModul(SyncModule* syncModul);
    void setAuth(AuthorityManager* auth);
    void setRoleAssignment(IRoleAssignment* roleAssignment);
    void setCommunicator(IAlicaCommunication* communicator);
    void setAlicaClock(AlicaClock* clock);
    void setResultStore(VariableSyncModule* resultStore);

    // internals
    void setStepCalled(bool stepCalled);
    bool getStepCalled() const;
    void iterationComplete();

    // AgentIDManager forwarded interface:

    AgentIDConstPtr getIdFromBytes(const std::vector<uint8_t>& vectorID) const;
    template <class Prototype>
    AgentIDConstPtr getId(Prototype& idPrototype) const;

private:
    bool hacked;
    void setStepEngine(bool stepEngine);

    PlanBase* planBase;
    TeamObserver* teamObserver;
    ExpressionHandler* expressionHandler;
    BehaviourPool* behaviourPool;
    const RoleSet* roleSet;
    VariableSyncModule* variableSyncModule;
    AuthorityManager* auth;
    TeamManager* _teamManager;
    SyncModule* syncModul;
    PlanRepository* planRepository;
    BlackBoard _blackboard;

    essentials::AgentIDManager* agentIDManager;
    Logger* log;
    PlanParser* planParser;

    IRoleAssignment* roleAssignment;
    IAlicaCommunication* communicator;
    AlicaClock* alicaClock;

    /**
     * Switch the engine between normal operation and silent mode, in which no messages other than debugging information
     * are sent out.
     * This is useful for a robot on hot standby.
     */
    bool _maySendMessages;
    /**
     * Set to have the engine's main loop wait on a signal via MayStep
     */
    bool stepEngine;
    /**
     * Indicates whether the engine is shutting down.
     */
    bool terminating;

    /**
     * Indicates whether the engine should run with a static role assignment
     * that is based on default roles, or not.
     */
    bool useStaticRoles;

    bool stepCalled;

    const Plan* masterPlan;
    std::unordered_map<size_t, ISolverBase*> _solvers;
    essentials::SystemConfig* sc;
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
AgentIDConstPtr AlicaEngine::getId(Prototype& idPrototype) const
{
    return AgentIDConstPtr(this->agentIDManager->getID<Prototype>(idPrototype));
}

template <typename T>
void AlicaEngine::abort(const std::string& msg, const T& tail)
{
    std::stringstream ss;
    ss << msg << tail;
    AlicaEngine::abort(ss.str());
}

template <class SolverType>
void AlicaEngine::addSolver(SolverType* solver)
{
    _solvers.emplace(typeid(SolverType).hash_code(), solver);
}

template <class SolverType>
SolverType* AlicaEngine::getSolver() const
{
    std::unordered_map<size_t, ISolverBase*>::const_iterator cit = _solvers.find(typeid(SolverType).hash_code());
    return (cit == _solvers.end()) ? nullptr : static_cast<SolverType*>(cit->second);
}

} // namespace alica
