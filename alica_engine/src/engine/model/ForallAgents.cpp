/*
 * ForallAgents.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/ForallAgents.h"
#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Plan.h"
#include "engine/Assignment.h"
#include "engine/teamobserver/TeamObserver.h"
#include "engine/AlicaEngine.h"
#include "engine/collections/StateCollection.h"
#include "engine/ITeamObserver.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/model/Variable.h"
#include <AutoDiff.h>

namespace alica
{

	ForallAgents::ForallAgents(long id) : Quantifier(id)
	{
		// TODO Auto-generated constructor stub

	}

	ForallAgents::~ForallAgents()
	{
		// TODO Auto-generated destructor stub
	}

	shared_ptr<list<vector<Variable*> > > ForallAgents::getSortedVariables(RunningPlan* p, shared_ptr<vector<int> > agentsInScope)
	{
		agentsInScope = nullptr;
		if(this->isScopeIsPlan())
		{
			if(p->getPlan() == this->getScopedPlan())
			{
				agentsInScope = p->getAssignment()->getAllRobotsSorted();
			}
		}
		else if(this->isScopeIsEntryPoint())
		{
			agentsInScope = p->getAssignment()->getRobotsWorkingSorted(this->getScopedEntryPoint());
		}
		else if(this->isScopeIsState())
		{
			agentsInScope = p->getAssignment()->getRobotStateMapping()->getRobotsInStateSorted(this->getScopedState());
		}
		if(agentsInScope == nullptr)
		{
			return nullptr;
		}
		shared_ptr<list<vector<Variable*> > > ret;
		ITeamObserver* to = AlicaEngine::getInstance()->getTeamObserver();
		for(int r : *(agentsInScope))
		{
			vector<Variable*> terms = vector<Variable*>(this->getDomainIdentifiers().size());
			RobotEngineData* re = to->getRobotById(r);
			for(int i = 0; i < terms.size(); i++)
			{

				auto iter = this->getDomainIdentifiers().begin();
				advance(iter, i);
				terms[i] = re->getSortedVariable(*iter);
			}
			ret->push_back(terms);
		}
		return ret;
	}

	shared_ptr<list<vector<AutoDiff::Term*> > > ForallAgents::getSortedTerms(RunningPlan* plan, shared_ptr<vector<int> > agentsInScope)
	{
		agentsInScope = nullptr;
		if(this->isScopeIsPlan())
		{
			if(plan->getPlan() == this->getScopedPlan())
			{
				agentsInScope = plan->getAssignment()->getAllRobotsSorted();
			}
		}
		else if(this->isScopeIsEntryPoint())
		{
			agentsInScope = plan->getAssignment()->getRobotsWorkingSorted(this->getScopedEntryPoint());
		}
		else if(this->isScopeIsState())
		{
			agentsInScope = plan->getAssignment()->getRobotStateMapping()->getRobotsInStateSorted(this->getScopedState());
		}
		if(agentsInScope == nullptr)
		{
			return nullptr;
		}
		shared_ptr<list<vector<AutoDiff::Term*> > > ret;
		ITeamObserver* to = AlicaEngine::getInstance()->getTeamObserver();
		for(int r : *(agentsInScope))
		{
			vector<AutoDiff::Term*> terms = vector<AutoDiff::Term*>(this->getDomainIdentifiers().size());
			RobotEngineData* re = to->getRobotById(r);
			for(int i = 0; i < terms.size(); i++)
			{

				auto iter = this->getDomainIdentifiers().begin();
				advance(iter, i);
				terms[i] = re->getSortedVariable(*iter)->getSolverVar();
			}
			ret->push_back(terms);
		}
		return ret;
	}


} /* namespace Alica */
