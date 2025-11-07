#pragma once

#include "engine/AlicaContext.h"
#include "engine/ConfigChangeListener.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/Types.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/blackboard/Blackboard.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/default/DefaultTransitionConditionCreator.h"
#include "engine/expressionhandler/ExpressionHandler.h"
#include "engine/modelmanagement/ModelManager.h"
#include "engine/syncmodule/SyncModule.h"
#include "engine/teammanager/TeamManager.h"

#include <list>
#include <string>
#include <unordered_map>

namespace alica
{
struct AlicaCreators;
class Plan;
class RoleSet;
class IRoleAssignment;
class VariableSyncModule;

class AlicaEngine
{
public:
    template <class... Args>
    static void abort(const std::string& logName, Args&&... args)
    {
        std::ostringstream oss;
        (oss << ... << std::forward<Args>(args));
        Logging::logFatal(logName) << "!!ABORT!! " << oss.str();
        std::abort();
    }

    AlicaEngine(AlicaContext& ctx, YAML::Node& config, const AlicaContextParams& alicaContextParams);
    ~AlicaEngine();

    // State modifiers:
    bool init(AlicaCreators&& creatorCtx);
    void start();
    void terminate();
    void stepNotify();

    // Parameter Access:
    // bool getStepEngine() const;
    bool maySendMessages() const { return _maySendMessages; }

    // Module Access:
    const AuthorityManager& getAuth() const { return _auth; }
    AuthorityManager& editAuth() { return _auth; }

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

    const std::shared_ptr<Blackboard> getGlobalBlackboard() { return _ctx.getGlobalBlackboardShared(); }

    // Data Access:
    const RoleSet* getRoleSet() const { return _roleSet; }
    uint64_t getMasterPlanId() const { return _masterPlan->getId(); }

    // internals
    void iterationComplete();
    int getVersion() const;

    // AlicaContext forwarded interface:
    const IAlicaCommunication& getCommunicator() const;
    const AlicaClock& getAlicaClock() const;
    IAlicaTimerFactory& getTimerFactory() const;
    // can be null if no traceFactory is set
    const IAlicaTraceFactory* getTraceFactory() const;
    std::string getLocalAgentName() const;
    template <class SolverType>
    SolverType& getSolver() const;
    template <class SolverType>
    bool existSolver() const;

    void reload(const YAML::Node& config);
    //[[deprecated("temporary method tobe removed in last PR")]]
    const YAML::Node& getConfig() const;

    /**
     * Call reload() of all subscribed components. Each component does reload using the
     * updated config.
     */
    void reloadConfig(const YAML::Node& config);     // to be removed in the last PR
    ConfigChangeListener& getConfigChangeListener(); // Used for test purpouse

private:
    static constexpr const char* LOGNAME = "AlicaEngine";

    // void setStepEngine(bool stepEngine);
    void initTransitionConditions(ITransitionConditionCreator* creator);
    std::atomic<bool> _stepEngine; /**< Set to have the engine's main loop wait on a signal via MayStep*/
    std::atomic<bool> _stepCalled; /**< Flag against spurious wakeups on the condition variable for step mode*/ // WARNING: Initialization order dependencies!
    // Please do not change the declaration order of members.
    ConfigChangeListener _configChangeListener;
    AlicaContext& _ctx;
    PlanRepository _planRepository;
    ModelManager _modelManager;
    const Plan* _masterPlan; /**< Pointing to the top level plan of the loaded ALICA program.*/
    const RoleSet* _roleSet; /**< Pointing to the current set of known roles.*/
    TeamManager _teamManager;
    SyncModule _syncModul;
    std::unique_ptr<IRoleAssignment> _roleAssignment;
    TeamObserver _teamObserver;
    ExpressionHandler _expressionHandler;
    AuthorityManager _auth;
    DefaultTransitionConditionCreator _defaultTransitionConditionCreator;

    /**
     * TODO: Make VariableSyncModule a stack variable.
     * Currently, it has circular dependency with engine header, because it needs to access
     * the ALICA clock via the engine in a template method in the VariableSyncModule-Header.
     * The clock cannot be stored in variable sync module, because it is changed at runtime via the
     * alica context interface. This happens, e.g., in some alica_tests cases.
     */
    std::unique_ptr<VariableSyncModule> _variableSyncModule;
    PlanBase _planBase;

    std::atomic<bool> _initialized{false};

    bool _useStaticRoles;  /**< Indicates whether the engine should run with a static role assignment that is based on default roles, or not. */
    bool _maySendMessages; /**< If false, engine sends only debugging information and does not participate in teamwork. Useful for hot standby. */
};

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
