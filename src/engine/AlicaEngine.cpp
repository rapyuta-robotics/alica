/*
 * AlicaEngine.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Stephan Opfer
 */
using namespace std;

#include "engine/AlicaEngine.h"

namespace alica
{

	/**
	 * The main class.
	 */
	AlicaEngine::AlicaEngine()
	{
		//Sanity check
		supplementary::SystemConfig::GetOwnRobotID();

		this->sc = supplementary::SystemConfig::getInstance();
		this->stepEngine = false;

		cout << "AE: Constructor called!" << endl;
	}

	AlicaEngine::~AlicaEngine()
	{
		// TODO Auto-generated destructor stub
	}

	void AlicaEngine::Init(string roleSetName, string masterPlanName, string roleSetDir, bool stepEngine)
	{
		this->SetStepEngine(stepEngine);

		this->planRepository = shared_ptr<PlanRepository> (new PlanRepository());
	}

	void AlicaEngine::Start()
	{
	}

	bool AlicaEngine::GetStepEngine()
	{
		return this->stepEngine;
	}

	void AlicaEngine::SetStepEngine(bool stepEngine)
	{
		this->stepEngine = stepEngine;
	}

} /* namespace Alica */
