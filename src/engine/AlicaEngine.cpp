/*
 * AlicaEngine.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Stephan Opfer
 */
//#define AE_DEBUG

using namespace std;

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/parser/PlanParser.h"
#include "engine/behaviourpool/BehaviourPool.h"
#include "engine/model/RoleSet.h"


namespace alica
{

	/**
	 * The main class.
	 */
	AlicaEngine::AlicaEngine()
	{
		this->behaviourPool = new BehaviourPool();
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

	void AlicaEngine::init(string roleSetName, string masterPlanName, string roleSetDir, bool stepEngine)
	{
		this->setStepEngine(stepEngine);

		this->planRepository = new PlanRepository();
		this->planParser = new PlanParser(this->planRepository);
		this->masterPlan = this->planParser->ParsePlanTree(masterPlanName);
		this->behaviourPool->init();
		this->roleSet = this->planParser->parseRoleSet(roleSetName, roleSetDir);
	}

	void AlicaEngine::start()
	{
	}

	bool AlicaEngine::getStepEngine()
	{
		return this->stepEngine;
	}

	unique_ptr<PlanRepository> AlicaEngine::getPlanRepository()
	{
		return unique_ptr<PlanRepository> (this->planRepository);
	}

	unique_ptr<IBehaviourPool> AlicaEngine::getBehaviourPool()
	{
		return unique_ptr<IBehaviourPool> (this->behaviourPool);
	}

	void AlicaEngine::setStepEngine(bool stepEngine)
	{
		this->stepEngine = stepEngine;
	}

	void AlicaEngine::abort (string msg){
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

	Logger* AlicaEngine::getLog() const
	{
		return log;
	}

	void AlicaEngine::setLog(Logger* log)
	{
		this->log = log;
	}

} /* namespace Alica */


