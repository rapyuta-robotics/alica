#pragma once

#include "engine/constraintmodul/ISolver.h"
#include "engine/blackboard/BlackBoard.h"
#include <SystemConfig.h>
#include <list>
#include <unordered_map>
#include <string>
#include <supplementary/AgentIDManager.h>

namespace supplementary {
class AgentIDFactory;
}

namespace alica {
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
class PlanSelector;
class PlanBase;
class ExpressionHandler;
class PartialAssignmentPool;
class VariableSyncModule;
class TeamManager;

class IBehaviourCreator;
class IUtilityCreator;
class IConditionCreator;
class IConstraintCreator;

class IAlicaCommunication;

class IRoleAssignment;

class AlicaEngine {
public:
    static void abort(const std::string& msg);
    template <typename T>
    static void abort(const std::string&, const T& tail);

    AlicaEngine(supplementary::AgentIDManager* idManager, const std::string& roleSetName, const std::string& masterPlanName, const std::string& roleSetDir,
            bool stepEngine);
    ~AlicaEngine();

    //State modifiers:
    bool init(IBehaviourCreator* bc, IConditionCreator* cc, IUtilityCreator* uc, IConstraintCreator* crc);
    void shutdown();
    void start();
    void stepNotify();

    //Parameter Access:
    bool isTerminating() const;
    bool getStepEngine() const;
    bool maySendMessages() const {return _maySendMessages;}
    std::string getRobotName() const;

    //Module Access:
    AuthorityManager* getAuth() const {return auth;}
    BehaviourPool* getBehaviourPool() const {return behaviourPool;}
    const IAlicaCommunication* getCommunicator() const {return communicator;}
    Logger* getLog() const {return log;}
    PartialAssignmentPool* getPartialAssignmentPool() const {return pap;}
    PlanBase* getPlanBase() const {return planBase;}
    PlanParser* getPlanParser() const {return planParser;}
    PlanRepository* getPlanRepository() const {return planRepository;}
    PlanSelector* getPlanSelector() const {return planSelector;}
    VariableSyncModule* getResultStore() const {return variableSyncModule;}
    IRoleAssignment* getRoleAssignment() const {return roleAssignment;}
    SyncModule* getSyncModul() const {return syncModul;}
    TeamManager* getTeamManager() const {return teamManager;}
    TeamObserver* getTeamObserver() const {return teamObserver;}
    AlicaClock* getAlicaClock() const {return alicaClock;}

    const BlackBoard& getBlackBoard() const {return _blackboard;}
    BlackBoard& editBlackBoard() {return _blackboard;}
    //Solver Access:
    template <class SolverType>
    void addSolver(SolverType* solver);
    template <class SolverType>
    SolverType* getSolver() const;

    //Data Access:

    const RoleSet* getRoleSet() const {return roleSet;}

    //Setters:
    void setMaySendMessages(bool maySendMessages);

    //Module Setters TODO: remove and replace with a better configuration system
    void setLog(Logger* log);
    void setTeamObserver(TeamObserver* teamObserver);
    void setSyncModul(SyncModule* syncModul);
    void setAuth(AuthorityManager* auth);
    void setRoleAssignment(IRoleAssignment* roleAssignment);
    void setCommunicator(IAlicaCommunication* communicator);
    void setAlicaClock(AlicaClock* clock);
    void setResultStore(VariableSyncModule* resultStore);

    //internals
    void setStepCalled(bool stepCalled);
    bool getStepCalled() const;
    void iterationComplete();

    //AgentIDManager forwarded interface:

    const supplementary::AgentID* getIDFromBytes(const std::vector<uint8_t>& vectorID);
    template <class Prototype>
    const supplementary::AgentID* getID(Prototype& idPrototype);

private:
    void setStepEngine(bool stepEngine);

    PlanBase* planBase;
    TeamObserver* teamObserver;
    ExpressionHandler* expressionHandler;
    BehaviourPool* behaviourPool;
    const RoleSet* roleSet;
    VariableSyncModule* variableSyncModule;
    AuthorityManager* auth;
    PlanSelector* planSelector;
    TeamManager* teamManager;
    PartialAssignmentPool* pap;
    SyncModule* syncModul;    
    PlanRepository* planRepository;
    BlackBoard _blackboard;
    
    supplementary::AgentIDManager* agentIDManager;
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
    supplementary::SystemConfig* sc;
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
const supplementary::AgentID* AlicaEngine::getID(Prototype& idPrototype) {
    return this->agentIDManager->getID<Prototype>(idPrototype);
}

template <typename T>
void AlicaEngine::abort(const std::string& msg, const T& tail) {
    std::stringstream ss;
    ss << msg << tail;
    AlicaEngine::abort(ss.str());
}

template <class SolverType>
void AlicaEngine::addSolver(SolverType* solver) {
    _solvers.emplace(typeid(SolverType).hash_code(), solver);
}

template <class SolverType>
SolverType* AlicaEngine::getSolver() const {
    std::unordered_map<size_t, ISolverBase*>::const_iterator cit = _solvers.find(typeid(SolverType).hash_code());
    return (cit == _solvers.end()) ? nullptr : static_cast<SolverType*>(cit->second);
}


}  // namespace alica
