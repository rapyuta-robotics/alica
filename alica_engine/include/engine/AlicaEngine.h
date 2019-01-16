#pragma once

#include "engine/AgentIDConstPtr.h"
#include "engine/blackboard/BlackBoard.h"
#include "engine/constraintmodul/ISolver.h"

#include <SystemConfig.h>
#include <essentials/AgentIDManager.h>
#include <list>
#include <string>

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

    AlicaEngine(essentials::AgentIDManager* idManager, const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine);
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
    AuthorityManager* getAuth() const { return _auth; }
    BehaviourPool* getBehaviourPool() const { return _behaviourPool; }
    const IAlicaCommunication* getCommunicator() const { return _communicator; }
    Logger* getLog() const { return _log; }
    PlanBase* getPlanBase() const { return _planBase; }
    PlanParser* getPlanParser() const { return _planParser; }
    PlanRepository* getPlanRepository() const { return _planRepository; }
    VariableSyncModule* getResultStore() const { return _variableSyncModule; }
    IRoleAssignment* getRoleAssignment() const { return _roleAssignment; }
    SyncModule* getSyncModul() const { return _syncModul; }
    TeamManager* getTeamManager() const { return _teamManager; }
    TeamObserver* getTeamObserver() const { return _teamObserver; }
    AlicaClock* getAlicaClock() const { return _alicaClock; }

    const BlackBoard& getBlackBoard() const { return _blackboard; }
    BlackBoard& editBlackBoard() { return _blackboard; }
    // Solver Access:
    template <class SolverType>
    void addSolver(SolverType* solver);
    template <class SolverType>
    SolverType* getSolver() const;

    // Data Access:

    const RoleSet* getRoleSet() const { return _roleSet; }

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
    void setStepEngine(bool stepEngine);

    PlanBase* _planBase;
    TeamObserver* _teamObserver;
    ExpressionHandler* _expressionHandler;
    BehaviourPool* _behaviourPool;
    const RoleSet* _roleSet;
    VariableSyncModule* _variableSyncModule;
    AuthorityManager* _auth;
    TeamManager* _teamManager;
    SyncModule* _syncModul;
    PlanRepository* _planRepository;
    BlackBoard _blackboard;

    essentials::AgentIDManager* _agentIDManager;
    Logger* _log;
    PlanParser* _planParser;

    IRoleAssignment* _roleAssignment;
    IAlicaCommunication* _communicator;
    AlicaClock* _alicaClock;

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
     * Indicates whether the engine is shutting down.
     */
    bool _terminating;

    /**
     * Indicates whether the engine should run with a static role assignment
     * that is based on default roles, or not.
     */
    bool _useStaticRoles;

    bool _stepCalled;

    const Plan* _masterPlan;
    std::unordered_map<size_t, ISolverBase*> _solvers;
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
    return AgentIDConstPtr(_agentIDManager->getID<Prototype>(idPrototype));
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
