/*
 * GSolver.h
 *
 *  Created on: Aug 12, 2014
 *      Author: psp
 */

#ifndef GSOLVER_H_
#define GSOLVER_H_

#include <AutoDiff.h>

#include <memory>
#include <vector>

#include <fstream>

using namespace std;
using namespace autodiff;

namespace alica
{
	namespace reasoner
	{
		class GSolver : public enable_shared_from_this<GSolver>
		{
		protected:
			class RpropResult;

		public:
			GSolver();

			vector<double> solve(shared_ptr<Term> equation, vector<shared_ptr<Variable>> args,
									vector<vector<double>> limits, double *util);
			bool solveSimple(shared_ptr<Term> equation, vector<shared_ptr<Variable>> args,
								vector<vector<double>> limits);
			vector<double> solve(shared_ptr<Term> equation, vector<shared_ptr<Variable>> args,
									vector<vector<double>> limits, vector<vector<double>> seeds,
									double sufficientUtility, double *util);
			bool solveSimple(shared_ptr<Term> equation, vector<shared_ptr<Variable>> args,
								vector<vector<double>> limits, vector<vector<double>> seeds);
			vector<double> solveTest(shared_ptr<Term> equation, vector<shared_ptr<Variable>> args,
										vector<vector<double>> limits);
			vector<double> solveTest(shared_ptr<Term> equation, vector<shared_ptr<Variable>> args,
										vector<vector<double>> limits, int maxRuns, bool *found);

			long getRuns();
			void setRuns(long runs);
			long getFEvals();
			void setFEvals(long fevals);
			long getMaxFEvals();
			void setMaxFEvals(long maxfevals);
			double getRPropConvergenceStepSize();
			void setRPropConvergenceStepSize(double rPropConvergenceStepSize);
		protected:
			static int _fcounter;
			bool _seedWithUtilOptimum;

			void initLog();
			void log(double util, vector<double> val);
			void logStep();
			void closeLog();
			vector<double> initialPointFromSeed(shared_ptr<RpropResult> res, vector<double> seed);
			vector<double> initialPoint(shared_ptr<RpropResult> res);
			shared_ptr<RpropResult> rPropLoop(vector<double> seed);
			shared_ptr<RpropResult> rPropLoop(vector<double> seed, bool precise);
			shared_ptr<RpropResult> rPropLoopSimple(vector<double> seed);
			void initialStepSize();
			bool evalResults();

			class RpropResult : public enable_shared_from_this<RpropResult>
			{
			public:
				vector<double> _initialValue;
				vector<double> _finalValue;
				double _initialUtil;
				double _finalUtil;
				bool _aborted;

				int compareTo(shared_ptr<RpropResult> other);
				double distanceTraveled();
				double distanceTraveledNormed(vector<double> ranges);

//				friend bool operator<(const shared_ptr<RpropResult>& left, const shared_ptr<RpropResult>& right);
//				friend bool operator>(const shared_ptr<RpropResult>& left, const shared_ptr<RpropResult>& right);
			};

		private:
			double _utilitySignificanceThreshold = 1E-22;
			// Random rand;
			int _dim;
			vector<vector<double>> _limits;
			vector<double> _ranges;
			vector<double> _rpropStepWidth;
			vector<double> _rpropStepConvergenceThreshold;
			double _utilityThreshold;
			ulong _maxSolveTime;
			// //vector<double>[] seeds;
			shared_ptr<ICompiledTerm> _term;

			ofstream sw;

			vector<shared_ptr<RpropResult>> _rResults;

			long _runs;
			long _fevals;
			long _maxfevals;
			double _initialStepSize = 0.005;
			double _rPropConvergenceStepSize;

		};
	} /* namespace reasoner */
} /* namespace alica */

#endif /* GSOLVER_H_ */
