#pragma once

#include <SystemConfig.h>
#include <list>
#include <map>
#include <string>
#include <supplementary/AgentIDManager.h>

namespace supplementary {
class AgentIDFactory;
}

namespace alica {
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
class IAlicaClock;

class ISolver;
class IRoleAssignment;

class AlicaEngine {
public:
    static void abort(string msg);
    template <typename T>
    static void abort(string msg, const T tail);

public:
    AlicaEngine(supplementary::AgentIDManager* idManager, string roleSetName, string masterPlanName, string roleSetDir,
            bool stepEngine);
    bool init(IBehaviourCreator* bc, IConditionCreator* cc, IUtilityCreator* uc, IConstraintCreator* crc);
    void shutdown();
    void start();
    bool getStepEngine();
    PlanRepository* getPlanRepository() const;
    BehaviourPool* getBehaviourPool();
    string getRobotName() const;
    Logger* getLog();
    void setLog(Logger* log);
    TeamObserver* getTeamObserver() const;
    void setTeamObserver(TeamObserver* teamObserver);

    void setSyncModul(SyncModule* syncModul);
    SyncModule* getSyncModul();
    AuthorityManager* getAuth();
    void setAuth(AuthorityManager* auth);
    IRoleAssignment* getRoleAssignment();
    void setRoleAssignment(IRoleAssignment* roleAssignment);
    PlanParser* getPlanParser() const;
    bool isTerminating() const;
    void setTerminating(bool terminating);
    void setStepCalled(bool stepCalled);
    bool getStepCalled() const;
    bool isMaySendMessages() const;
    void setMaySendMessages(bool maySendMessages);
    RoleSet* getRoleSet();
    const IAlicaCommunication* getCommunicator() const;
    void setCommunicator(IAlicaCommunication* communicator);
    PlanSelector* getPlanSelector();
    IAlicaClock* getIAlicaClock() const;
    void setIAlicaClock(IAlicaClock* clock);
    void iterationComplete();
    PartialAssignmentPool* getPartialAssignmentPool() const;
    void stepNotify();
    PlanBase* getPlanBase();
    void addSolver(int identifier, ISolver* solver);
    ISolver* getSolver(int identifier);
    VariableSyncModule* getResultStore();
    void setResultStore(VariableSyncModule* resultStore);
    TeamManager* getTeamManager() const;

    const supplementary::AgentID* getIDFromBytes(const std::vector<uint8_t>& vectorID);

    template <class Prototype>
    const supplementary::AgentID* getID(Prototype& idPrototype);

    ~AlicaEngine();

    /**
     * Switch the engine between normal operation and silent mode, in which no messages other than debugging information
     * are sent out.
     * This is useful for a robot on hot standby.
     */
    bool maySendMessages;

protected:
    Logger* log;
    RoleSet* roleSet;
    SyncModule* syncModul;
    AuthorityManager* auth;
    ExpressionHandler* expressionHandler;
    PlanSelector* planSelector;
    TeamManager* teamManager;
    PartialAssignmentPool* pap;
    PlanBase* planBase;
    VariableSyncModule* variableSyncModule;
    PlanRepository* planRepository;
    PlanParser* planParser;
    BehaviourPool* behaviourPool;
    TeamObserver* teamObserver;
    supplementary::AgentIDManager* agentIDManager;

    IRoleAssignment* roleAssignment;
    IAlicaCommunication* communicator;
    IAlicaClock* alicaClock;

private:
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

    supplementary::SystemConfig* sc;
    bool stepCalled;
    Plan* masterPlan;
    map<int, ISolver*> solver;

    void setStepEngine(bool stepEngine);
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
void AlicaEngine::abort(string msg, const T tail) {
    stringstream ss;
    ss << msg << tail;
    AlicaEngine::abort(ss.str());
}

}  // namespace alica
