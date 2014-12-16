/*
 * CNSMTGSolver.h
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#ifndef CNSMTGSOLVER_H_
#define CNSMTGSOLVER_H_

#include <AutoDiff.h>
#include <engine/IAlicaClock.h>

#include <memory>
#include <vector>

#include <fstream>

using namespace std;
using namespace autodiff;

namespace autodiff
{
	class Term;
	class Variable;
} // namespace autodiff

namespace alica
{
	class IAlicaClock;

	namespace reasoner
	{
		namespace intervalpropagation
		{
			class IntervalPropagator;
		} // namespace intervalpropagation

		namespace cnsat
		{
			class CNSat;
			class FormulaTransform;
			class Var;
		} // namespace cnsat

		class CNSMTGSolver : public enable_shared_from_this<CNSMTGSolver>
		{
		protected:
			class RpropResult;

		public:
			CNSMTGSolver();
			virtual ~CNSMTGSolver();

			void initLog();
			void log(double util, shared_ptr<vector<double>>& val);
			void logStep();
			void closeLog();
			shared_ptr<vector<double>> solve(shared_ptr<Term> equation,
												shared_ptr<vector<shared_ptr<autodiff::Variable>> > args,
												shared_ptr<vector<shared_ptr<vector<double>> > >& limits, double *util);
			shared_ptr<vector<double>> solve(shared_ptr<Term> equation,
												shared_ptr<vector<shared_ptr<autodiff::Variable>> > args,
												shared_ptr<vector<shared_ptr<vector<double>> > >& limits,
												shared_ptr<vector<shared_ptr<vector<double>> > > seeds,
												double sufficientUtility, double *util);
			shared_ptr<vector<double>> solveTest(shared_ptr<Term> equation,
													shared_ptr<vector<shared_ptr<autodiff::Variable>> > args,
													shared_ptr<vector<shared_ptr<vector<double>> > >& limits);
			bool intervalPropagate(shared_ptr<vector<shared_ptr<cnsat::Var>> > decisions,
									shared_ptr<vector<shared_ptr<vector<double>> > >& curRanges);
			bool probeForSolution(shared_ptr<vector<shared_ptr<cnsat::Var>> > decisions,
									shared_ptr<vector<double>> solution);
			bool currentCacheConsistent();

			long getRuns();
			long getFEvals();
			double getRPropConvergenceStepSize();

			double utilitySignificanceThreshold = 1E-22;
			int dim;
			shared_ptr<vector<shared_ptr<vector<double>> > > limits;

			double utilityThreshold;
			ulong maxSolveTime;
			alicaTime begin;

			ofstream sw;

			vector<shared_ptr<RpropResult>> rResults;

			long maxfevals;
			double initialStepSize = 0.005;
			bool useIntervalProp;
			bool optimize;

			IAlicaClock* alicaClock;

		protected:
			shared_ptr<RpropResult> rPropFindFeasible(shared_ptr<vector<shared_ptr<cnsat::Var>> > constraints,
														shared_ptr<vector<double>> seed);
			shared_ptr<RpropResult> rPropOptimizeFeasible(shared_ptr<vector<shared_ptr<cnsat::Var>> > constraints,
															shared_ptr<autodiff::Term> ut,
															shared_ptr<vector<shared_ptr<autodiff::Variable>> > args,
															shared_ptr<vector<double>>& seed, bool precise);
			void differentiate(shared_ptr<vector<shared_ptr<cnsat::Var>> > constraints, shared_ptr<vector<double>> val,
								shared_ptr<vector<double>> gradient, double *util);
			shared_ptr<vector<double>> initialPointFromSeed(shared_ptr<vector<shared_ptr<cnsat::Var>> > constraints,
															shared_ptr<RpropResult> res,
															shared_ptr<vector<double>>& seed);
			shared_ptr<vector<double>> initialPoint(shared_ptr<vector<shared_ptr<cnsat::Var>> > constraints,
													shared_ptr<RpropResult> res);
			void initializeStepSize();

			long runs;
			long fevals;
			double rPropConvergenceStepSize;

			static int fcounter;
			bool seedWithUtilOptimum;
			shared_ptr<cnsat::CNSat> ss;

			shared_ptr<cnsat::FormulaTransform> ft;
			shared_ptr<intervalpropagation::IntervalPropagator> ip;
			shared_ptr<vector<double>> lastSeed;

			shared_ptr<RpropResult> r1 = nullptr;

			int probeCount = 0;
			int successProbeCount = 0;
			int intervalCount = 0;
			int successIntervalCount = 0;
			int fevalsCount = 0;
			int runCount = 0;

			vector<double> ranges;
			vector<double> rpropStepWidth;
			vector<double> rpropStepConvergenceThreshold;
			shared_ptr<vector<shared_ptr<autodiff::Variable> > > currentArgs;

			class RpropResult : public enable_shared_from_this<RpropResult>
			{
			public:
				shared_ptr<vector<double>> initialValue;
				shared_ptr<vector<double>> finalValue;
				double initialUtil;
				double finalUtil;
				bool aborted;

				int compareTo(shared_ptr<RpropResult> other);
			};
		};

	} /* namespace reasoner */
} /* namespace alica */

#endif /* CNSMTGSOLVER_H_ */
