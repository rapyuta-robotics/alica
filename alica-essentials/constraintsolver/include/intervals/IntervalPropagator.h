/*
 * IntervalPropagator.h
 *
 *  Created on: Dec 5, 2014
 *      Author: Philipp
 */

#ifndef INTERVALPROPAGATOR_H_
#define INTERVALPROPAGATOR_H_

#include <AutoDiff.h>

#include <memory>

using namespace std;
using namespace autodiff;

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{
			class CNSat;
			class Var;
		}

		namespace intervalpropagation
		{
			class RecursivePropagate;
			class ResetIntervals;

			class IntervalPropagator
			{
			public:
				IntervalPropagator();
				virtual ~IntervalPropagator();

				static int updates;
				static int visits;

				void setGlobalRanges(shared_ptr<vector<shared_ptr<Variable>>> vars, shared_ptr<vector<shared_ptr<vector<double>>>> ranges, shared_ptr<cnsat::CNSat> solver);
				bool propagate(shared_ptr<vector<shared_ptr<cnsat::Var>>> decisions, shared_ptr<vector<shared_ptr<vector<double>>>>& completeRanges, shared_ptr<vector<shared_ptr<cnsat::Var>>>& offenders);
				bool prePropagate(shared_ptr<vector<shared_ptr<cnsat::Var>>> vars);
				bool propagate(shared_ptr<Term> term);

			protected:
				shared_ptr<ResetIntervals> ri;
				shared_ptr<RecursivePropagate> rp;

				shared_ptr<vector<shared_ptr<vector<double> > > > globalRanges;
				shared_ptr<vector<shared_ptr<Variable> > > vars;
				int dim;
				shared_ptr<cnsat::CNSat> solver;

				bool propagateSingle(shared_ptr<cnsat::Var> v, bool sign);
		};

	}
		/* namespace intervalpropagation */
		} /* namespace reasoner */
	} /* namespace alica */

#endif /* INTERVALPROPAGATOR_H_ */
