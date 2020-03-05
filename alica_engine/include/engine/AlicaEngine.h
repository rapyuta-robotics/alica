#pragma once

#include "engine/AlicaContext.h"
#include "engine/BehaviourPool.h"
#include "engine/Logger.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/modelmanagement/ModelManager.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/blackboard/BlackBoard.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/expressionhandler/ExpressionHandler.h"
#include "engine/syncmodule/SyncModule.h"
#include "engine/teammanager/TeamManager.h"

#include <essentials/SystemConfig.h>
#include <essentials/IdentifierConstPtr.h>
#include <essentials/IDManager.h>

#include <list>
#include <string>
#include <unordered_map>

namespace alica
{
struct AlicaCreators;
class Plan;
class BehaviourPool;
class Logger;
class RoleSet;
class VariableSyncModule;
class IRoleAssignment;

class AlicaEngine
{
public:
    static void abort(const std::string& msg);
    template <typename T>
    static void abort(const std::string&, const T& tail);

    // TODO: Put idManager* into AlicaContext
    AlicaEngine(AlicaContext& ctx, const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine);
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

    const ModelManager& getModelManager() const { return _modelManager; }
    ModelManager& editModelManager() {return _modelManager; }

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

    // AlicaContext forwarded interface:
    const IAlicaCommunication& getCommunicator() const;
    const AlicaClock& getAlicaClock() const;
    std::string getLocalAgentName() const;
    template <class SolverType>
    SolverType& getSolver() const;
    template <class SolverType>
    bool existSolver() const;
    essentials::IdentifierConstPtr getIDFromBytes(const uint8_t *idBytes, int idSize, uint8_t type = essentials::Identifier::UUID_TYPE) const;
    essentials::IdentifierConstPtr getIdFromBytes(const std::vector<uint8_t>& vectorID) const;
    template <class Prototype>
    essentials::IdentifierConstPtr getID(Prototype& idPrototype) const;
    essentials::IdentifierConstPtr generateId(std::size_t size) const;

private:
    void setStepEngine(bool stepEngine);
    // WARNING: Initialization order dependencies, do not change the declaration order of members.
    // TODO: Check with regard of the warning...
    AlicaContext& _ctx;
    PlanRepository _planRepository;
    ModelManager _modelManager;
    TeamManager _teamManager;
    TeamObserver _teamObserver;
    BehaviourPool _behaviourPool;
    SyncModule _syncModul;
    ExpressionHandler _expressionHandler;
    AuthorityManager _auth;
    PlanBase _planBase;
    BlackBoard _blackboard;
    Logger _log;
    // TODO: fix this, VariableSyncModule has circular dependency with engine header
    // VariableSyncModule _variableSyncModule;
    std::unique_ptr<VariableSyncModule> _variableSyncModule;
    std::unique_ptr<IRoleAssignment> _roleAssignment;

    /**
     * Pointing to the top level plan of the loaded ALICA program.
     */
    const Plan* _masterPlan;
    /**
     * Pointing to the current set of known roles.
     */
    const RoleSet* _roleSet;
    /**
     * Indicates whether the engine should run with a static role assignment
     * that is based on default roles, or not.
     */
    bool _useStaticRoles;
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
    bool _stepCalled;
};

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing an int and receiving
 * a pointer to a corresponding Identifier object.
 */
template <class Prototype>
essentials::IdentifierConstPtr AlicaEngine::getID(Prototype& idPrototype) const
{
    return _ctx.getID<Prototype>(idPrototype);
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
