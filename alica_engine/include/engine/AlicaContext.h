/**
 * @file
 * This file contains api interface of alica for applications.
 */

#pragma once

#include "engine/IAlicaTimer.h"
#include "engine/IAlicaTrace.h"
#include "engine/IAlicaWorldModel.h"
#include "engine/IBehaviourCreator.h"
#include "engine/IConditionCreator.h"
#include "engine/IConstraintCreator.h"
#include "engine/IPlanCreator.h"
#include "engine/IUtilityCreator.h"
#include "engine/Types.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/util/ConfigPathParser.h"
#include "engine/syncmodule/SyncModule.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/TeamObserver.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/constraintmodul/VariableSyncModule.h"

#include <alica_common_config/debug_output.h>

#include <cassert>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace alica
{

class AlicaEngine;
class IAlicaCommunication;
class AlicaTestsEngineGetter;

namespace test
{
class TestContext;
}

/**
 * Alica creators that framework uses to instantiate various behaviours, utilities, conditions or constraints.
 */
struct AlicaCreators
{
    AlicaCreators(std::unique_ptr<IConditionCreator> cc, std::unique_ptr<IUtilityCreator> uc, std::unique_ptr<IConstraintCreator> crc,
            std::unique_ptr<IBehaviourCreator> bc, std::unique_ptr<IPlanCreator> pc)
            : conditionCreator(std::move(cc))
            , utilityCreator(std::move(uc))
            , constraintCreator(std::move(crc))
            , behaviourCreator(std::move(bc))
            , planCreator(std::move(pc))
    {
        assert(conditionCreator && utilityCreator && constraintCreator && behaviourCreator && planCreator);
    }

    std::unique_ptr<IConditionCreator> conditionCreator;
    std::unique_ptr<IUtilityCreator> utilityCreator;
    std::unique_ptr<IConstraintCreator> constraintCreator;
    std::unique_ptr<IBehaviourCreator> behaviourCreator;
    std::unique_ptr<IPlanCreator> planCreator;
};

/**
 * Struct containing all necessary params for creating an AlicaContext object.
 */
struct AlicaContextParams
{
    /**
     * @param agentName Name of the local agent.
     * @param configPath Path to the configuration folder.
     * @param roleSetName Name of the roleSet.
     * @param masterPlanName Name of the masterPlan
     * @param stepEngine Signify engine is trigger based. Defaults to false.
     * @param agentID Identifier of the local Agent. If no identifier is given,
     * the engine will try to read the local agent's identifier from the
     * Alica.yaml config instead. If no identifier is specified in the
     * config as well, the engine will generate a random identifier.
     *
     * @note The configPath is the path containing the plans, roles and tasks folder.
     */
    AlicaContextParams(const std::string& agentName, const std::string& configPath, const std::string& roleSetName, const std::string& masterPlanName,
            bool stepEngine = false, const AgentId agentID = InvalidAgentID)
            : agentName(agentName)
            , configPath(configPath)
            , roleSetName(roleSetName)
            , masterPlanName(masterPlanName)
            , stepEngine(stepEngine)
            , agentID(agentID)
    {
    }

    /**
     * @param agentName Name of the local agent.
     * @param configPath Path to the configuration folder.
     * @param agentID Identifier of the local Agent. If no identifier is given,
     * the engine will try to read the local agent's identifier from the
     * Alica.yaml config instead. If no identifier is specified in the
     * config as well, the engine will generate a random identifier.
     *
     * @note The configPath is the path containing the plans, roles and tasks folder.
     */
    AlicaContextParams(const std::string& agentName, const std::string& configPath, const AgentId agentID = InvalidAgentID)
            : agentName(agentName)
            , configPath(configPath)
            , roleSetName("RoleSet")
            , masterPlanName("MasterPlan")
            , stepEngine(false)
            , agentID(agentID)
    {
    }

    std::string agentName;
    std::string configPath;
    std::string masterPlanName;
    std::string roleSetName;
    bool stepEngine;
    AgentId agentID;
};

/*
 * @class AlicaContext
 *
 * @brief AlicaContext class is an interface class to alica. AlicaContext object encapsulates all the global state
 * associated with this instance of framework. An application must instantiate and use this class to launch alica.
 */
class AlicaContext
{
public:
    /**
     * Get host (or agent) name for this process.
     *
     * @return The agent name under which the engine operates, a string
     */
    std::string getLocalAgentName() const;

    /**
     * Get version of Alica engine.
     *
     * @param[out] major Version major
     * @param[out] minor Version minor
     * @param[out] patch Version patch
     */
    static void getVersion(int& major, int& minor, int& patch);

    /**
     * Get version of Alica engine.
     *
     * @return compressed representation of sdk version
     */
    static int getVersion();

    /**
     * Creates AlicaContext object.
     *
     * @param alicaContextParams Struct containing all necessary params to create the Context.
     *
     * @note This is the main alica api class
     * @note Can throw an exception if some necessary configurations dont exist in the config file.
     */
    AlicaContext(const AlicaContextParams& alicaContextParams);

    /**
     * Destroys AlicaContext object.
     */
    ~AlicaContext();

    AlicaContext(const AlicaContext& other) = delete;
    AlicaContext& operator=(const AlicaContext& other) = delete;

    /**
     * Initialize alica framework and related modules.
     *
     * @param creatorCtx Creator functions for utility, behaviour, constraint and condition
     *
     * @return Return code '0' stands for success, any other for corresponding error
     *
     * @see AlicaCreators
     */
    int init(AlicaCreators& creatorCtx);

    /**
     * Terminate alica framework and related modules. This function must be called for safe termination before
     * deleting Context.
     *
     * @param creatorCtx Creator functions for utility, behaviour, constraint and condition
     *
     * @return Return code '0' stands for success, any other for corresponding error
     */
    int terminate();

    /**
     * Set Alica Clock choose between RosClock or systemClock (std::chrono)
     *
     * @note ClockType must be a derived class of AlicaClock
     * @note This must be called before initializing context
     */
    template <class ClockType, class... Args>
    void setClock(Args&&... args);

    /**
     * Get clock being used by this alica instance.
     *
     * @return A reference to alica clock object being used by context
     */
    const AlicaClock& getAlicaClock() const
    {
        assert(_clock.get());
        return *_clock;
    }

    /**
     * Set communicator to be used by this alica framework instance.
     * Example usage: setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
     *
     * @note CommunicatorType must be a derived class of IAlicaCommunication
     * @note This must be called before initializing context
     *
     * @param args Arguments to be forwarded to constructor of communicator. Might be empty.
     */
    template <class CommunicatorType, class... Args>
    void setCommunicator(Args&&... args);

    /**
     * Get communicator being used by this alica instance.
     *
     * @return A reference to communicator object being used by context
     */
    const IAlicaCommunication& getCommunicator() const
    {
        assert(_communicator.get());
        return *_communicator;
    }

    /**
     * Set world model to be used by this alica framework instance.
     * Example usage: setWorldModel<alicaDummyProxy::alicaDummyWorldModel>();
     *
     * @note WorldModelType must be a derived class of IAlicaWorldModel
     * @note This must be called before initializing context
     *
     * @param args Arguments to be forwarded to constructor of world model. Might be empty.
     */
    template <class WorldModelType, class... Args>
    void setWorldModel(Args&&... args);

    /**
     * Get worldModel being used by this alica instance. If no worldModel has been set,
     * the returned value will be a nullptr.
     *
     * @return A pointer to worldModel object being used by context
     */
    IAlicaWorldModel* getWorldModel() const { return _worldModel.get(); }

    /**
     * Add a solver to be used by this alica instance.
     * Example usage: addSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
     *
     * @note SolverType must be a derived class of ISolverBase
     *
     * @param args Arguments to be forwarded to constructor of solver. Might be empty.
     */
    template <class SolverType, class... Args>
    void addSolver(Args&&... args);

    /**
     * Get a particular solver from alica instance.
     * Example usage: getSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
     *
     * @note If in doubt, check whether solver exists with existSolver() method
     *
     * @return A reference to requested solver instance
     */
    template <class SolverType>
    SolverType& getSolver() const;

    /**
     * Check whether a particular solver is associated with this alica instance.
     * Example usage: existSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
     *
     * @return True if the solver exist, false otherwise
     */
    template <class SolverType>
    bool existSolver() const;

    /**
     * Set timer factory to be used by this alica framework instance.
     * Example usage: setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>();
     *
     * @note TimerFactoryType must be a derived class of IAlicaTimerFactory
     * @note This must be called before initializing context
     *
     * @param args Arguments to be forwarded to constructor of timer factory. Might be empty.
     */
    template <class TimerFactoryType, class... Args>
    void setTimerFactory(Args&&... args);

    /**
     * Get timer factory being used by this alica instance.
     *
     * @return A reference to timer factory object being used by context
     */
    IAlicaTimerFactory& getTimerFactory() const
    {
        assert(_timerFactory.get());
        return *_timerFactory;
    }

    /**
     * Set trace factory to be used by this alica framework instance.
     * Example usage: setTraceFactory<amr_tracing::TraceFactory>();
     *
     * @note TraceFactoryType must be a derived class of IAlicaTraceFactory
     * @note This must be called before initializing context
     *
     * @param args Arguments to be forwarded to constructor of trace factory. Might be empty.
     */
    template <class TraceFactoryType, class... Args>
    void setTraceFactory(Args&&... args);

    /**
     * Get trace factory being used by this alica instance.
     *
     * @return A reference to trace factory object being used by context
     */
    IAlicaTraceFactory* getTraceFactory() const { return _traceFactory.get(); }

    /**
     * Check whether object is a valid AlicaContext.
     *
     * @return True if object is a valid context, false otherwise
     */
    bool isValid() const;

    /**
     * Returns agent id for this alica context.
     *
     * @return Object representing id of local agent.
     */
    AgentId getLocalAgentId() const;

    /**
     * Execute one step of engine synchronously
     */
    void stepEngine();

    /**
     * Getter for the agents configuration.
     *
     * @return const YAML::Node& containing the agents configuration.
     */
    const YAML::Node& getConfig() const { return _configRootNode; };

    /**
     * Set config values for the agent.
     *
     * @param path Path of the config value.
     * @param value Value to set in the config.
     * @param reload Reload all subscribed components, defaults to true
     *
     * @note Example path: "Alica.CycleDetection.Enabled"
     * @note Use '.' to access subsections and values of a config section.
     *
     * @return True if value was set correctly. False otherwise.
     */
    template <class T>
    bool setOption(const std::string& path, const T& value, bool reload = true) noexcept;

    /**
     * Set config values for the agent. On error no config values are updated.
     *
     * @param keyValuePairs Vector of key value pairs
     * @param path Path of the config value.
     * @param value Value to set in the config.
     * @param reload Reload all subscribed components, defaults to true
     *
     * @note Example path: "Alica.CycleDetection.Enabled"
     * @note Use '.' to access subsections and values of a config section.
     *
     * @return True if values were set correctly. False otherwise.
     */
    template <class T>
    bool setOptions(const std::vector<std::pair<std::string, T>>& keyValuePairs, bool reload = true) noexcept;

private:
    friend class ::alica::AlicaTestsEngineGetter;
    friend class ::alica::test::TestContext;

    std::string _localAgentName;
    YAML::Node _configRootNode;

    uint32_t _validTag;
    // WARNING: Initialization order dependencies!
    // Please do not change the declaration order of members.
    std::unique_ptr<AlicaClock> _clock;
    std::unique_ptr<IAlicaCommunication> _communicator;
    std::unique_ptr<AlicaEngine> _engine;
    std::unordered_map<size_t, std::unique_ptr<ISolverBase>> _solvers;
    std::unique_ptr<IAlicaTimerFactory> _timerFactory;
    std::unique_ptr<IAlicaTraceFactory> _traceFactory;
    std::unique_ptr<IAlicaWorldModel> _worldModel;

    bool _initialized = false;

    /**
     * Initializes yaml configuration.
     * @param configPath Relative path to the yaml configuration file.
     * @param agentName Name of the local agent.
     *
     * @return The agents config.
     */
    YAML::Node initConfig(const std::string& configPath, const std::string& agentName);

    /**
     * Reload alica components with updated configuration.
     *
     * @note Is called when setOption or setOptions is successfully called.
     */
    void reloadConfig();
};

template <class ClockType, class... Args>
void AlicaContext::setClock(Args&&... args)
{
    static_assert(std::is_base_of<AlicaClock, ClockType>::value, "Must be derived from AlicaClock");
#if (defined __cplusplus && __cplusplus >= 201402L)
    _clock = std::make_unique<ClockType>(std::forward<Args>(args)...);
#else
    _clock = std::unique_ptr<ClockType>(new ClockType(std::forward<Args>(args)...));
#endif
}

template <class CommunicatorType, class... Args>
void AlicaContext::setCommunicator(Args&&... args)
{
    static_assert(std::is_base_of<IAlicaCommunication, CommunicatorType>::value, "Must be derived from IAlicaCommunication");
#if (defined __cplusplus && __cplusplus >= 201402L)
    _communicator = std::make_unique<CommunicatorType>(
        std::bind(&SyncModule::onSyncTalk, _engine.get()->editSyncModul(), std::placeholders::_1),
        std::bind(&SyncModule::onSyncReady, _engine.get()->editSyncModul(), std::placeholders::_1),
        std::bind(&AuthorityManager::handleIncomingAuthorityMessage, _engine.get()->editAuth(), std::placeholders::_1),
        std::bind(&TeamObserver::handlePlanTreeInfo, _engine.get()->editTeamObserver(), std::placeholders::_1),
        std::bind(&VariableSyncModule::onSolverResult, _engine.get()->editResultStore(), std::placeholders::_1),
        std::bind(&TeamManager::handleAgentQuery, _engine.get()->getTeamManager(), std::placeholders::_1),
        std::bind(&TeamManager::handleAgentAnnouncement, _engine.get()->editTeamManager(), std::placeholders::_1),
        std::forward<Args>(args)...
    );
#else
    _communicator = std::unique_ptr<CommunicatorType>(new CommunicatorType(_engine.get(), std::forward<Args>(args)...));
#endif
}

template <class SolverType, class... Args>
void AlicaContext::addSolver(Args&&... args)
{
    static_assert(std::is_base_of<ISolverBase, SolverType>::value, "Must be derived from ISolverBase");
#if (defined __cplusplus && __cplusplus >= 201402L)
    _solvers.emplace(typeid(SolverType).hash_code(), std::make_unique<SolverType>(_engine.get(), std::forward<Args>(args)...));
#else
    _solvers.emplace(typeid(SolverType).hash_code(), std::unique_ptr<SolverType>(new SolverType(_engine.get(), std::forward<Args>(args)...)));
#endif
}

template <class SolverType>
SolverType& AlicaContext::getSolver() const
{
    auto cit = _solvers.find(typeid(SolverType).hash_code());
    assert(cit != _solvers.end());
    return static_cast<SolverType&>(*(cit->second));
}

template <class SolverType>
bool AlicaContext::existSolver() const
{
    auto cit = _solvers.find(typeid(SolverType).hash_code());
    return (cit != _solvers.end());
}

template <class TimerFactoryType, class... Args>
void AlicaContext::setTimerFactory(Args&&... args)
{
    static_assert(std::is_base_of<IAlicaTimerFactory, TimerFactoryType>::value, "Must be derived from IAlicaTimerFactory");
#if (defined __cplusplus && __cplusplus >= 201402L)
    _timerFactory = std::make_unique<TimerFactoryType>(std::forward<Args>(args)...);
#else
    _timerFactory = std::unique_ptr<TimerFactoryType>(new TimerFactoryType(std::forward<Args>(args)...));
#endif
}

template <class TraceFactoryType, class... Args>
void AlicaContext::setTraceFactory(Args&&... args)
{
    static_assert(std::is_base_of<IAlicaTraceFactory, TraceFactoryType>::value, "Must be derived from IAlicaTraceFactory");
#if (defined __cplusplus && __cplusplus >= 201402L)
    _traceFactory = std::make_unique<TraceFactoryType>(std::forward<Args>(args)...);
#else
    _traceFactory = std::unique_ptr<TraceFactoryType>(new TraceFactoryType(std::forward<Args>(args)...));
#endif
}

template <class WorldModelType, class... Args>
void AlicaContext::setWorldModel(Args&&... args)
{
    static_assert(std::is_base_of<IAlicaWorldModel, WorldModelType>::value, "Must be derived from IAlicaWorldModel");
#if (defined __cplusplus && __cplusplus >= 201402L)
    _worldModel = std::make_unique<WorldModelType>(std::forward<Args>(args)...);
#else
    _worldModel = std::unique_ptr<WorldModelType>(new WorldModelType(std::forward<Args>(args)...));
#endif
}

template <class T>
bool AlicaContext::setOption(const std::string& path, const T& value, bool reload) noexcept
{
    if (_initialized) {
        return false;
    }
    ConfigPathParser configPathParser;
    std::vector<std::string> params = configPathParser.getParams('.', path);

    try {
        YAML::Node currentNode(_configRootNode);

        for (const std::string& param : params) {
            currentNode.reset(currentNode[param]);
        }
        currentNode = value;
    } catch (const YAML::Exception& e) {
        ALICA_WARNING_MSG("AC: Could not set config value: " << e.msg);
        return false;
    }

    if (reload) {
        reloadConfig();
    }
    return true;
}

template <class T>
bool AlicaContext::setOptions(const std::vector<std::pair<std::string, T>>& keyValuePairs, bool reload) noexcept
{
    if (_initialized) {
        return false;
    }
    ConfigPathParser configPathParser;
    std::vector<std::pair<std::string, T>> oldKeyValuePairs;

    try {
        for (const auto& keyValuePair : keyValuePairs) {
            std::vector<std::string> params = configPathParser.getParams('.', keyValuePair.first);
            YAML::Node currentNode(_configRootNode);

            for (const std::string& param : params) {
                currentNode.reset(currentNode[param]);
            }

            T oldValue = currentNode.as<T>();
            currentNode = keyValuePair.second;

            std::pair<std::string, T> oldKeyValuePair = std::make_pair(keyValuePair.first, oldValue);
            oldKeyValuePairs.push_back(oldKeyValuePair);
        }
    } catch (const YAML::Exception& e) {
        ALICA_WARNING_MSG("AC: Could not set config values: " << e.msg);
        // revert changes
        for (const auto& keyValuePair : oldKeyValuePairs) {
            setOption<T>(keyValuePair.first, keyValuePair.second, false);
        }
        return false;
    }

    if (reload) {
        reloadConfig();
    }

    return true;
}

} // namespace alica
