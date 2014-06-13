/*
 * AlicaEngine.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Stephan Opfer
 */
using namespace std;

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/parser/PlanParser.h"
#include "engine/model/RoleSet.h"

namespace alica
{

	/**
	 * The main class.
	 */
	AlicaEngine::AlicaEngine()
	{

		this->sc = supplementary::SystemConfig::getInstance();
		this->stepEngine = false;

		cout << "AE: Constructor finished!" << endl;
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

		this->planRepository = shared_ptr<PlanRepository>(new PlanRepository());
		this->planParser = shared_ptr<IPlanParser>(new PlanParser(this->planRepository));
		this->masterPlan = this->planParser->parsePlanTree(masterPlanName);
		this->roleSet = this->planParser->parseRoleSet(roleSetName, roleSetDir);
	}

	void AlicaEngine::start()
	{
	}

	bool AlicaEngine::getStepEngine()
	{
		return this->stepEngine;
	}

	shared_ptr<PlanRepository> AlicaEngine::getPlanRepository()
	{
		return this->planRepository;
	}

	void AlicaEngine::setStepEngine(bool stepEngine)
	{
		this->stepEngine = stepEngine;
	}
	void AlicaEngine::abort (string msg){
		cerr << "ABORT: " << msg << endl;
		exit(EXIT_FAILURE);
	}


} /* namespace Alica */
