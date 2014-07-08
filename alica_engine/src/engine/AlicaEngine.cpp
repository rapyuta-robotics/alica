/*
 * AlicaEngine.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Stephan Opfer
 */
#define AE_DEBUG

using namespace std;

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/parser/PlanParser.h"
#include "engine/behaviourpool/BehaviourPool.h"
#include "engine/model/RoleSet.h"
#include "engine/ISyncModul.h"
#include "engine/IRoleAssignment.h"
#include "engine/allocationauthority/AuthorityManager.h"

namespace alica
{

	/**
	 * The main class.
	 */
	AlicaEngine::AlicaEngine()
	{
		this->roleSet = nullptr;
		this->masterPlan = nullptr;
		this->planParser = nullptr;
		this->teamObserver = nullptr;
		this->log = nullptr;
		this->planRepository = nullptr;
		this->syncModul = nullptr;
		this->roleAssignment = nullptr;
		this->auth = nullptr;
		this->behaviourPool = nullptr;
		this->roleSet = nullptr;
		this->sc = supplementary::SystemConfig::getInstance();
		this->stepEngine = false;
#ifdef AE_DEBUG
		cout << "AE: Constructor finished!" << endl;
#endif
	}

	AlicaEngine::~AlicaEngine()
	{
	}

	/**
	 * The method for getting the singleton instance.
	 * @return A pointer to the AlicaEngine object, you must not delete.
	 */
	AlicaEngine* AlicaEngine::getInstance()
	{
		static AlicaEngine instance;
		return &instance;
	}

	bool AlicaEngine::init(IBehaviourCreator* bc, string roleSetName, string masterPlanName, string roleSetDir,
	bool stepEngine)
	{
		bool everythingWorked = true;
		this->setStepEngine(stepEngine);
		this->planRepository = new PlanRepository();
		this->planParser = new PlanParser(this->planRepository);
		this->masterPlan = this->planParser->parsePlanTree(masterPlanName);
		this->roleSet = this->planParser->parseRoleSet(roleSetName, roleSetDir);
		this->behaviourPool = new BehaviourPool();
		everythingWorked &= this->behaviourPool->init(bc);
		return everythingWorked;
	}

	bool AlicaEngine::shutdown()
	{
		bool everythingWorked = true;
		delete this->planRepository;
		this->planRepository = nullptr;
		delete this->planParser;
		this->planParser = nullptr;
		// roleSet is an Element in the elements set of the model factory and is cleaned there already
		//delete this->roleSet;
		this->roleSet= nullptr;
		delete this->behaviourPool;
		this->behaviourPool = nullptr;
		return everythingWorked;
	}

	void AlicaEngine::start()
	{
	}

	bool AlicaEngine::getStepEngine()
	{
		return this->stepEngine;
	}

	PlanRepository* AlicaEngine::getPlanRepository()
	{
		return this->planRepository;
	}

	IBehaviourPool* AlicaEngine::getBehaviourPool()
	{
		return this->behaviourPool;
	}

	ITeamObserver* AlicaEngine::getTeamObserver()
	{
		return this->teamObserver;
	}

	void AlicaEngine::setTeamObserver(ITeamObserver* teamObserver)
	{
		this->teamObserver = teamObserver;
	}

	ISyncModul* AlicaEngine::getSyncModul()
	{
		return syncModul;
	}

	void AlicaEngine::setSyncModul(ISyncModul* syncModul)
	{
		this->syncModul = syncModul;
	}

	AuthorityManager* AlicaEngine::getAuth()
	{
		return auth;
	}

	void AlicaEngine::setAuth(AuthorityManager* auth)
	{
		this->auth = auth;
	}

	IRoleAssignment* AlicaEngine::getRoleAssignment()
	{
		return roleAssignment;
	}

	void AlicaEngine::setRoleAssignment(IRoleAssignment* roleAssignment)
	{
		this->roleAssignment = roleAssignment;
	}

	IPlanParser* AlicaEngine::getPlanParser()
	{
		return planParser;
	}

	RoleSet* AlicaEngine::getRoleSet()
	{
		return roleSet;
	}

	void AlicaEngine::setStepEngine(bool stepEngine)
	{
		this->stepEngine = stepEngine;
	}

	void AlicaEngine::abort(string msg)
	{
		cerr << "ABORT: " << msg << endl;
		exit(EXIT_FAILURE);
	}

	const string& AlicaEngine::getRobotName() const
	{
		return robotName;
	}

	void AlicaEngine::setRobotName(const string& robotName)
	{
		this->robotName = robotName;
	}

	Logger* AlicaEngine::getLog()
	{
		return log;
	}

	void AlicaEngine::setLog(Logger* log)
	{
		this->log = log;
	}

	bool AlicaEngine::isTerminating() const
	{
		return terminating;
	}

	void AlicaEngine::setTerminating(bool terminating)
	{
		this->terminating = terminating;
	}

	IAlicaCommunication* AlicaEngine::getCommunicatior()
	{
		return communicatior;
	}

} /* namespace Alica */


