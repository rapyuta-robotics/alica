/*
 * ForallAgents.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef FORALLAGENTS_H_
#define FORALLAGENTS_H_

#include <list>
#include <vector>
#include <memory>


#include "Quantifier.h"

namespace AutoDiff
{
	class Term;
}

namespace alica
{

	class RunningPlan;
	class Variable;

	/**
	 * A quantifier associated with agents, i.e., the domain identifiers of this quantifier refer to properties of an agent
	 */
	class ForallAgents : public Quantifier
	{
	public:
		ForallAgents(long id = 0);
		virtual ~ForallAgents();
		shared_ptr<list<vector<Variable* > > > getSortedVariables(RunningPlan* p, shared_ptr<vector<int> > agentsInScope);
		shared_ptr<list<vector<AutoDiff::Term*> > > getSortedTerms(RunningPlan* p, shared_ptr<vector<int> > agentsInScope);


	};

} /* namespace Alica */

#endif /* FORALLAGENTS_H_ */
