#pragma once

#include "engine/AlicaContext.h"
#include "engine/Logger.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/RuntimeBehaviourFactory.h"
#include "engine/RuntimePlanFactory.h"
#include "engine/TeamObserver.h"
#include "engine/Types.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/blackboard/Blackboard.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/expressionhandler/ExpressionHandler.h"
#include "engine/modelmanagement/ModelManager.h"
#include "engine/syncmodule/SyncModule.h"
#include "engine/teammanager/TeamManager.h"

#include <functional>
#include <list>
#include <string>
#include <unordered_map>

namespace alica
{
struct AlicaCreators;
class Plan;
class Logger;
class RoleSet;
class IRoleAssignment;
class VariableSyncModule;

using ReloadFunction=std::function<void(const YAML::Node&)>;
using SubscribeFunction=std::function<void(ReloadFunction)>;

class AlicaEngine
{
public:
    static void abort(const std::string& msg);
    template <typename T>
    static void abort(const std::string&, const T& tail);

    AlicaEngine(AlicaContext& ctx, const std::string& configPath, const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine,
            const AgentId agentID = InvalidAgentID);
    ~AlicaEngine();

    // State modifiers:
    bool init(AlicaCreators&& creatorCtx);
    void start();
    void terminate();
    void stepNotify();

    // Parameter Access:
    bool getStepEngine() const;
    bool maySendMessages() const { return _maySendMessages; }

    // Module Access:
    const AuthorityManager& getAuth() const { return _auth; }
    AuthorityManager& editAuth() { return _auth; }

    const RuntimeBehaviourFactory& getRuntimeBehaviourFactory() const { return *_behaviourFactory; }
    const RuntimePlanFactory& getRuntimePlanFactory() const { return *_planFactory; }

    const Logger& getLog() const { return _log; }
    Logger& editLog() { return _log; }

    const PlanBase& getPlanBase() const { return _planBase; }
    PlanBase& editPlanBase() { return _planBase; }

    const ModelManager& getModelManager() const { return _modelManager; }
    ModelManager& editModelManager() { return _modelManager; }

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

    const Blackboard& getBlackboard() const { return _Blackboard; }
    Blackboard& editBlackboard() { return _Blackboard; }

    // Data Access:
    const RoleSet* getRoleSet() const { return _roleSet; }
    const uint64_t getMasterPlanId() const { return _masterPlan->getId(); }

    // internals
    void setStepCalled(bool stepCalled);
    bool getStepCalled() const;
    void iterationComplete();
    int getVersion() const;

    // AlicaContext forwarded interface:
    const IAlicaCommunication& getCommunicator() const;
    const AlicaClock& getAlicaClock() const;
    IAlicaTimerFactory& getTimerFactory() const;
    // can be null if no traceFactory is set
    const IAlicaTraceFactory* getTraceFactory() const;
    IAlicaWorldModel* getWorldModel() const;
    std::string getLocalAgentName() const;
    template <class SolverType>
    SolverType& getSolver() const;
    template <class SolverType>
    bool existSolver() const;
    AgentId generateID();

    void reload(const YAML::Node& config);
    const YAML::Node& getConfig() const;
    void subscribe(std::function<void(const YAML::Node& config)> reloadFunction);

    /**
     * Call reload() of all subscribed components. Each component does reload using the
     * updated config.
     */
    void reloadConfig(const YAML::Node& config);

private:
    void setStepEngine(bool stepEngine);
    // WARNING: Initialization order dependencies!
    // Please do not change the declaration order of members.
    std::vector<std::function<void(const YAML::Node& config)>> _configChangeListenerCBs;
    AlicaContext& _ctx;
    PlanRepository _planRepository;
    ModelManager _modelManager;
    const Plan* _masterPlan; /**< Pointing to the top level plan of the loaded ALICA program.*/
    const RoleSet* _roleSet; /**< Pointing to the current set of known roles.*/
    TeamManager _teamManager;
    TeamObserver _teamObserver;
    SyncModule _syncModul;
    ExpressionHandler _expressionHandler;
    AuthorityManager _auth;
    Logger _log;
    std::unique_ptr<IRoleAssignment> _roleAssignment;
    std::unique_ptr<RuntimeBehaviourFactory> _behaviourFactory;
    std::unique_ptr<RuntimePlanFactory> _planFactory;
    PlanBase _planBase;

    /**
     * TODO: Make VariableSyncModule a stack variable.
     * Currently, it has circular dependency with engine header, because it needs to access
     * the ALICA clock via the engine in a template method in the VariableSyncModule-Header.
     * The clock cannot be stored in variable sync module, because it is changed at runtime via the
     * alica context interface. This happens, e.g., in some alica_tests cases.
     */
    std::unique_ptr<VariableSyncModule> _variableSyncModule;
    Blackboard _Blackboard;
    bool _useStaticRoles;  /**< Indicates whether the engine should run with a static role assignment that is based on default roles, or not. */
    bool _maySendMessages; /**< If false, engine sends only debugging information and does not participate in teamwork. Useful for hot standby. */
    bool _stepEngine;      /**< Set to have the engine's main loop wait on a signal via MayStep*/
    bool _stepCalled;      /**< Flag against spurious wakeups on the condition variable for step mode*/
};

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
