/*
 * RuleBook.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#include "engine/rules/RuleBook.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include "engine/RunningPlan.h"
#include "engine/ITeamObserver.h"

#include <SystemConfig.h>

namespace alica
{
	RuleBook::RuleBook()
	{
		AlicaEngine* ae = AlicaEngine::getInstance();
		this->to = ae->getTeamObserver();
		this->ps = ae->getPlanSelector();
		this->sm = ae->getSyncModul();
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->maxConsecutiveChanges = (*sc)["Alica"]->get<int>("Alica.MaxRuleApplications", NULL);
		this->changeOccured = true;

	}

	RuleBook::~RuleBook()
	{
		// TODO Auto-generated destructor stub
	}


	RunningPlan* RuleBook::initialisationRule(Plan* masterPlan)
	{
		if(masterPlan->getEntryPoints().size() != 1)
		{
			AlicaEngine::getInstance()->abort("RB: Masterplan does not have exactly one task!");
		}

		RunningPlan* main = new RunningPlan(masterPlan);
		main->setAssignment(new Assignment(masterPlan));

		main->setAllocationNeeded(true);
		unique_ptr<list<int> > robots = to->getAvailableRobotIds();
		main->setRobotsAvail(move(robots));
		//TODO::
		return main;

	}


	/** Getter and Setter **/
	bool RuleBook::isChangeOccured() const
	{
		return changeOccured;
	}

	void RuleBook::setChangeOccured(bool changeOccured)
	{
		this->changeOccured = changeOccured;
	}
}
