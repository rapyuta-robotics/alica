/**
 * @file
 * This file contains api interface of alica for applications.
 */

#pragma once

#include "engine/IBehaviourCreator.h"
#include "engine/IConditionCreator.h"
#include "engine/IConstraintCreator.h"
#include "engine/IUtilityCreator.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/util/PathParser.h"

#include <essentials/IDManager.h>

#include <cassert>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace essentials
{
class IdentifierConstPtr;
} // namespace essentials

namespace alica
{

class AlicaEngine;
class IAlicaCommunication;
class AlicaTestsEngineGetter;

namespace test {
    class TestContext;
}

/**
 * Alica creators that framework uses to instantiate various behaviours, utilities, conditions or constraints.
 */
struct AlicaCreators
{
    AlicaCreators(std::unique_ptr<IConditionCreator> cc, std::unique_ptr<IUtilityCreator> uc, std::unique_ptr<IConstraintCreator> crc,
            std::unique_ptr<IBehaviourCreator> bc)
            : conditionCreator(std::move(cc))
            , utilityCreator(std::move(uc))
            , constraintCreator(std::move(crc))
            , behaviourCreator(std::move(bc))
    {
        assert(conditionCreator && utilityCreator && constraintCreator && behaviourCreator);
    }

    std::unique_ptr<IConditionCreator> conditionCreator;
    std::unique_ptr<IUtilityCreator> utilityCreator;
    std::unique_ptr<IConstraintCreator> constraintCreator;
    std::unique_ptr<IBehaviourCreator> behaviourCreator;
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
    std::string getLocalAgentName();

    /**
     * Set host (or agent) name for this process.
     *
     * @param name Host name
     */
    void setLocalAgentName(const std::string& name);

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
     * @param roleSetName Name for roleset
     * @param masterPlanName Name for the main plan
     * @param stepEngine Signify engine is trigger based.
     *
     * @note This is the main alica api class
     */
    AlicaContext(const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine,
                 const essentials::Identifier& agentID = essentials::Identifier());

    /**
     * Creates AlicaContext object.
     *
     * @note This is the main alica api class
     */
    AlicaContext();

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
     * Set id manager to be used by this alica framework instance.
     * Example usage: setIDManager<essentials::IDManager>();
     *
     * @note Currently, the only implementation is essentials::IDManager
     * @note This must be called before initializing context
     *
     * @param args Arguments to be forwarded to constructor of communicator. Might be empty.
     */
    template <class IDManagerType, class... Args>
    void setIDManager(Args&&... args);

    /**
     * Get id manager being used by this alica instance.
     *
     * @note Currently, the only implementation is essentials::IDManager
     * @return A reference to id manager object being used by context
     */
    essentials::IDManager& getIDManager() const
    {
        assert(_idManager.get());
        return *_idManager;
    }

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
    essentials::IdentifierConstPtr getLocalAgentId() const;

    /**
     * Execute one step of engine synchronously
     */
    void stepEngine();

    /**
     * Initializes yaml configuration.
     * @param configPath Relative path to the yaml configuration file, defaults to /etc.
     */
    void initConfig(std::string configPath = "./etc/Alica.yaml");

    const YAML::Node& getConfig() const
    {
        return _configRootNode;
    };

    const std::string getConfigPath() const
    {
        return _configPath;
    };

    AlicaEngine* getEngine() const
    {
        return _engine.get();
    };

    template<class T>
    void setOption(std::string& path, T value);

    template<class T>
    void setOption(std::vector<std::pair<std::string, T>> keyValuePairs);

    void buildObjects(const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine,
              const std::string& fullConfigPath, const essentials::Identifier& agentID = essentials::Identifier());

private:
    friend class ::alica::AlicaTestsEngineGetter;
    friend class ::alica::test::TestContext;

    uint32_t _validTag;
    // WARNING: Initialization order dependencies!
    // Please do not change the declaration order of members.
    std::unique_ptr<AlicaClock> _clock;
    std::unique_ptr<IAlicaCommunication> _communicator;
    std::unique_ptr<essentials::IDManager> _idManager;
    std::unique_ptr<AlicaEngine> _engine;
    std::unordered_map<size_t, std::unique_ptr<ISolverBase>> _solvers;

    YAML::Node _configRootNode;
    std::string _configPath;
    bool _initialized;
    std::string _localAgentName;

    template <class T>
    void setOption(YAML::Node node, std::vector<std::string> params, T value, unsigned int depth);
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
    _communicator = std::make_unique<CommunicatorType>(_engine.get(), std::forward<Args>(args)...);
#else
    _communicator = std::unique_ptr<CommunicatorType>(new CommunicatorType(_engine.get(), std::forward<Args>(args)...));
#endif
}

template <class IDManagerType, class... Args>
void AlicaContext::setIDManager(Args&&... args)
{
#if (defined __cplusplus && __cplusplus >= 201402L)
    _idManager = std::make_unique<IDManagerType>(std::forward<Args>(args)...);
#else
    _idManager = std::unique_ptr<IDManagerType>(new IDManagerType(std::forward<Args>(args)...));
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

template <class T>
void AlicaContext::setOption(std::string& path, T value)
{
    if (_initialized) {
        return;
    }
    PathParser pathParser;
    std::vector<std::string> params = pathParser.getParams('.', path.c_str());
    unsigned int depth = 0;
    setOption(_configRootNode, params, value, depth);
}

template <class T>
void AlicaContext::setOption(YAML::Node node, std::vector<std::string> params, T value, unsigned int depth)
{
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it){
        if (params[depth] == it->first.as<std::string>()){
            if (depth == params.size() - 1) {
                node = value;
            } else {
                setOption(it->second, params, value, depth + 1);
            }
        }
    }
}

template <class T>
void AlicaContext::setOption(std::vector<std::pair<std::string, T>> keyValuePairs)
{
    if (_initialized) {
        return;
    }

    for (int i = 0; i < keyValuePairs.size(); i++) {
        setOption<T>(keyValuePairs.get(0).first, keyValuePairs.get(1).second);
    }
}

} // namespace alica