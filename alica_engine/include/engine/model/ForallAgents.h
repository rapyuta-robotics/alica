#pragma once

#include "Quantifier.h"
#include "engine/IRobotID.h"

#include <list>
#include <vector>
#include <memory>



namespace alica
{

	class RunningPlan;
	class Variable;
	class AlicaEngine;
	class SolverTerm;

	/**
	 * A quantifier associated with agents, i.e., the domain identifiers of this quantifier refer to properties of an agent
	 */
	class ForallAgents : public Quantifier
	{
	public:
		ForallAgents(AlicaEngine* ae, long id = 0);
		virtual ~ForallAgents();
		shared_ptr<list<vector<Variable* > > > getDomainVariables(shared_ptr<RunningPlan>& p, shared_ptr<vector<const alica::IRobotID*> >& agentsInScope);

	protected:
		AlicaEngine* ae;


	};

} /* namespace Alica */
