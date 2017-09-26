#include "engine/model/ForallAgents.h"
#include "engine/teammanager/Agent.h"
#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/Assignment.h"
#include "engine/ITeamManager.h"
#include "engine/AlicaEngine.h"
#include "engine/collections/StateCollection.h"
#include "engine/ITeamObserver.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/model/Variable.h"
#include "engine/constraintmodul/SolverTerm.h"
#include "engine/constraintmodul/SolverVariable.h"

namespace alica
{

	ForallAgents::ForallAgents(AlicaEngine* ae, long id) :
			Quantifier(id)
	{
		this->ae = ae;
	}

	ForallAgents::~ForallAgents()
	{
	}

	/**
	 * Returns the <see cref="Variable"/>s currently associated with the agents occupying the scope of this quantifier.
	 * @param plan A RunningPlan
	 * @param agentsInScope A shared_ptr<vector<int> >
	 * @return shared_ptr<list<vector<Variable*> > >
	 */
	shared_ptr<list<vector<Variable*> > > ForallAgents::getDomainVariables(shared_ptr<RunningPlan>& p,
																			shared_ptr<vector<const alica::IRobotID*> >& agentsInScope)
	{
		if (this->isScopeIsPlan())
		{
			if (p->getPlan() == this->getScopedPlan())
			{
				agentsInScope = p->getAssignment()->getAllRobotsSorted();
			}
		}
		else if (this->isScopeIsEntryPoint())
		{
			agentsInScope = p->getAssignment()->getRobotsWorkingSorted(this->getScopedEntryPoint());
		}
		else if (this->isScopeIsState())
		{
			agentsInScope = p->getAssignment()->getRobotStateMapping()->getRobotsInStateSorted(this->getScopedState());
		}
		if (agentsInScope == nullptr)
		{
			return nullptr;
		}
		auto ret = make_shared<list<vector<Variable*>>>();
		ITeamManager* tm = ae->getTeamManager();
		for (auto& r : *(agentsInScope))
		{
			vector<Variable*> terms = vector<Variable*>(this->getDomainIdentifiers().size());
			auto robotEngineData = tm->getAgentByID(r)->getEngineData();
			for (int i = 0; i < terms.size(); i++)
			{
				auto iter = this->getDomainIdentifiers().begin();
				advance(iter, i);
				terms[i] = robotEngineData->getDomainVariable(*iter);
			}
			ret->push_back(terms);
		}
		return ret;
	}
} /* namespace Alica */
