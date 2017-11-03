#pragma once

#include <SystemConfig.h>
#include <list>
#include <map>
#include <string>
#include <supplementary/AgentIDManager.h>

using namespace std;

namespace supplementary
{
class IAgentIDFactory;
}
namespace alica
{
class PlanRepository;
class Plan;
class IPlanParser;
class IBehaviourPool;
class Logger;
class RoleSet;
class ITeamObserver;
class IBehaviourCreator;
class ISyncModul;
class AuthorityManager;
class IRoleAssignment;
class IPlanSelector;
class IAlicaCommunication;
class IEngineModule;
class IPlanner;
class IAlicaClock;
class PlanBase;
class IConditionCreator;
class IConstraintCreator;
class IUtilityCreator;
class ExpressionHandler;
class PartialAssignmentPool;
class ISolver;
class IVariableSyncModule;
class ITeamManager;

class AlicaEngine
{
  public:
    AlicaEngine();
    bool init(IBehaviourCreator *bc, IConditionCreator *cc, IUtilityCreator *uc, IConstraintCreator *crc,
              supplementary::AgentIDManager *idManager, string roleSetName, string masterPlanName, string roleSetDir,
              bool stepEngine);
    void shutdown();
    void start();
    bool getStepEngine();
    void abort(string msg) const;
    template <typename T>
    void abort(string msg, const T tail) const;
    PlanRepository *getPlanRepository() const;
    IBehaviourPool *getBehaviourPool();
    string getRobotName() const;
    Logger *getLog();
    void setLog(Logger *log);
    ITeamObserver *getTeamObserver() const;
    void setTeamObserver(ITeamObserver *teamObserver);

    void setSyncModul(ISyncModul *syncModul);
    ISyncModul *getSyncModul();
    AuthorityManager *getAuth();
    void setAuth(AuthorityManager *auth);
    IRoleAssignment *getRoleAssignment();
    void setRoleAssignment(IRoleAssignment *roleAssignment);
    IPlanParser *getPlanParser() const;
    bool isTerminating() const;
    void setTerminating(bool terminating);
    void setStepCalled(bool stepCalled);
    bool getStepCalled() const;
    bool isMaySendMessages() const;
    void setMaySendMessages(bool maySendMessages);
    RoleSet *getRoleSet();
    const IAlicaCommunication *getCommunicator() const;
    void setCommunicator(IAlicaCommunication *communicator);
    IPlanSelector *getPlanSelector();
    IPlanner *getPlanner();
    IAlicaClock *getIAlicaClock() const;
    void setIAlicaClock(IAlicaClock *clock);
    void iterationComplete();
    PartialAssignmentPool *getPartialAssignmentPool() const;
    void stepNotify();
    PlanBase *getPlanBase();
    void addSolver(int identifier, ISolver *solver);
    ISolver *getSolver(int identifier);
    IVariableSyncModule *getResultStore();
    void setResultStore(IVariableSyncModule *resultStore);
    ITeamManager *getTeamManager() const;


	const supplementary::IAgentID* getIDFromBytes(const std::vector<uint8_t> &vectorID);

	template <class Prototype>
	const supplementary::IAgentID* getID(Prototype &idPrototype);

    ~AlicaEngine();

    /**
     * Switch the engine between normal operation and silent mode, in which no messages other than debugging information
     * are sent out.
     * This is useful for a robot on hot standby.
     */
    bool maySendMessages;

  protected:
    supplementary::SystemConfig *sc;
    Plan *masterPlan;
    Logger *log;
    RoleSet *roleSet;
    ISyncModul *syncModul;
    AuthorityManager *auth;
    IRoleAssignment *roleAssignment;
    ExpressionHandler *expressionHandler;
    list<IEngineModule *> mods;
    IPlanSelector *planSelector;
    IAlicaCommunication *communicator;
    supplementary::IAgentIDFactory *robotIDFactory;
    IPlanner *planner;
    IAlicaClock *alicaClock;
    ITeamManager *teamManager;
    PartialAssignmentPool *pap;
    PlanBase *planBase;
    bool stepCalled;
    map<int, ISolver *> solver;
    IVariableSyncModule *variableSyncModule;
    supplementary::AgentIDManager *agentIDManager;

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
    void setStepEngine(bool stepEngine);

    PlanRepository *planRepository;
    IPlanParser *planParser;
    IBehaviourPool *behaviourPool;
    ITeamObserver *teamObserver;
};

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing an int and receiving
 * a pointer to a corresponding IAgentID object (in that case an
 * IntRobotID).
 */
template <class Prototype>
const supplementary::IAgentID* AlicaEngine::getID(Prototype &idPrototype)
{
    return this->agentIDManager->getID<Prototype>(idPrototype);
}

template <typename T>
void AlicaEngine::abort(string msg, const T tail) const
{
    stringstream ss;
    ss << msg << tail;
    AlicaEngine::abort(ss.str());
}

} /* namespace Alica */
