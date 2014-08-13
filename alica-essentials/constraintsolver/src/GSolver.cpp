/*
 * GSolver.cpp
 *
 *  Created on: Aug 12, 2014
 *      Author: psp
 */

#include "GSolver.h"

//#define GSOLVER_LOG
//#define ALWAYS_CHECK_THRESHOLD
#define AGGREGATE_CONSTANTS

#include "SystemConfig.h"
#include "Configuration.h"

#include <limits>
#include <string>
#include <sstream>
#include <stdlib.h>     /* srand, rand */
#include <cmath>
#include <time.h>       /* time */

#include <iostream>

namespace Alica
{
	namespace Reasoner
	{
		int GSolver::_fcounter = 0;

		GSolver::GSolver()
		{
			AutoDiff::Term::setAnd(AutoDiff::AndType::AND);
			AutoDiff::Term::setOr(AutoDiff::OrType::MAX);

			// this.rand = new Random();
			_rResults = vector<shared_ptr<RpropResult>>();

			_seedWithUtilOptimum = true;
			supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
			_maxfevals = (*sc)["Alica"]->get<int>("Alica", "CSPSolving", "MaxFunctionEvaluations", NULL);
			_maxSolveTime = ((ulong)(*sc)["Alica"]->get<int>("Alica", "CSPSolving", "MaxSolveTime", NULL)) * 1000000;
//			_maxfevals = 100000000;
//			_maxSolveTime = 25 * 1000000;
			_rPropConvergenceStepSize = 1E-2;
		}

		void GSolver::initLog()
		{
			std::stringstream ss;
			ss << "/tmp/test" << (_fcounter++) << ".dbg";
			string logFile = ss.str();
			sw.open(logFile);
		}

		void GSolver::log(double util, vector<double> val)
		{
			sw << util;
			sw << "\t";
			for (int i = 0; i < _dim; ++i)
			{
				sw << val[i];
				sw << "\t";
			}
			sw << endl;
		}

		void GSolver::logStep()
		{
			sw << endl;
			sw << endl;
		}

		void GSolver::closeLog()
		{
			sw.close();
		}

		vector<double> GSolver::solve(shared_ptr<AutoDiff::Term> equation, vector<shared_ptr<AutoDiff::Variable>> args,
										vector<vector<double>> limits, double *util)
		{
			return solve(equation, args, limits, vector<vector<double>>(), numeric_limits<double>::max(), util);
		}

		bool GSolver::solveSimple(shared_ptr<AutoDiff::Term> equation, vector<shared_ptr<AutoDiff::Variable>> args,
									vector<vector<double>> limits)
		{
			return solveSimple(equation, args, limits, vector<vector<double>>());
		}

		vector<double> GSolver::solve(shared_ptr<AutoDiff::Term> equation, vector<shared_ptr<AutoDiff::Variable>> args,
										vector<vector<double>> limits, vector<vector<double>> seeds,
										double sufficientUtility, double *util)
		{
			_fevals = 0;
			_runs = 0;
			*util = 0;
			_utilityThreshold = sufficientUtility;

#ifdef GSOLVER_LOG
			initLog();
#endif

			_rResults.clear();
			ulong begin = time(NULL);

			_dim = args.size();
			_limits = limits;
			_ranges = vector<double>(_dim);
			for (int i = 0; i < _dim; ++i)
			{
				_ranges[i] = (_limits[i][1] - _limits[i][0]);
			}
#ifdef AGGREGATE_CONSTANTS
			equation = equation->aggregateConstants();
#endif
			_term = AutoDiff::TermUtils::compile(equation, args);
			shared_ptr<AutoDiff::ConstraintUtility> cu = dynamic_pointer_cast<AutoDiff::ConstraintUtility>(equation);
			bool utilIsConstant = dynamic_pointer_cast<AutoDiff::Constant>(cu->getUtility()) != 0;
			if (utilIsConstant)
			{
				_utilityThreshold = 0.75;
			}
			bool constraintIsConstant = dynamic_pointer_cast<AutoDiff::Constant>(cu->getConstraint()) != 0;
			if (constraintIsConstant)
			{
				shared_ptr<AutoDiff::Constant> constraint = dynamic_pointer_cast<AutoDiff::Constant>(
						cu->getConstraint());
				if (constraint->getValue() < 0.25)
				{
					*util = constraint->getValue();
					vector<double> ret = vector<double>(_dim);
					for (int i = 0; i < _dim; ++i)
					{
						ret[i] = _ranges[i] / 2.0 + _limits[i][0];
					}
					return ret;
				}
			}

			//Optimize given seeds
			_rpropStepWidth = vector<double>(_dim);
			_rpropStepConvergenceThreshold = vector<double>(_dim);
			if (seeds.size() > 0)
			{
				_runs++;
				//Run with prefered cached seed
				shared_ptr<RpropResult> rpfirst = rPropLoop(seeds[0], true);
				if (rpfirst->_finalUtil > _utilityThreshold)
				{
					*util = rpfirst->_finalUtil;
					return rpfirst->_finalValue;
				}
				_rResults.push_back(rpfirst);
				//run with seeds of all other agends
				for (int i = 0; i < seeds.size(); ++i)
				{
					if (begin + _maxSolveTime < time(NULL) || _fevals > _maxfevals)
					{
						break; //do not check any further seeds
					}
					_runs++;
					shared_ptr<RpropResult> rp = rPropLoop(seeds[i], false);
					if (rp->_finalUtil > _utilityThreshold)
					{
						*util = rp->_finalUtil;
						return rp->_finalValue;
					}
					_rResults.push_back(rp);
				}
			}

			//Here: Ignore all constraints search optimum
			if (begin + _maxSolveTime > time(NULL) && _fevals < _maxfevals)
			{
				//if time allows, do an unconstrained run
				if (!constraintIsConstant && !utilIsConstant && _seedWithUtilOptimum)
				{
					shared_ptr<AutoDiff::ICompiledTerm> curProb = _term;
					_term = AutoDiff::TermUtils::compile(cu->getUtility(), args);
					_runs++;
					vector<double> utilitySeed = rPropLoop(vector<double>())->_finalValue;
					_term = curProb;
					//Take result and search with constraints
					shared_ptr<RpropResult> ru = rPropLoop(utilitySeed, false);
					/*if (ru->_finalUtil > _utilityThreshold) {
					 *util = ru->_finalUtil;
					 return ru->_finalValue;
					 }*/
					_rResults.push_back(ru);
				}
			}

			do
			{ //Do runs until termination criteria, running out of time, or too many function evaluations
				_runs++;
				shared_ptr<RpropResult> rp = rPropLoop(vector<double>(), false);
				if (rp->_finalUtil > _utilityThreshold)
				{
					*util = rp->_finalUtil;
					return rp->_finalValue;
				}
				_rResults.push_back(rp);
			} while (begin + _maxSolveTime > time(NULL) && _fevals < _maxfevals);

			//return best result
			int resIdx = 0;
			shared_ptr<RpropResult> res = _rResults[0];
			for (int i = 1; i < _rResults.size(); ++i)
			{
				if (std::isnan(res->_finalUtil) || _rResults[i]->_finalUtil > res->_finalUtil)
				{
					if (resIdx == 0 && seeds.size() > 0 && !std::isnan(res->_finalUtil))
					{
						if (_rResults[i]->_finalUtil - res->_finalUtil > _utilitySignificanceThreshold
								&& _rResults[i]->_finalUtil > 0.75)
						{
							res = _rResults[i];
							resIdx = i;
						}
					}
					else
					{
						res = _rResults[i];
						resIdx = i;
					}
				}
			}

#ifdef GSOLVER_LOG
			closeLog();
#endif

			*util = res->_finalUtil;
			return res->_finalValue;
		}

		bool GSolver::solveSimple(shared_ptr<AutoDiff::Term> equation, vector<shared_ptr<AutoDiff::Variable>> args,
									vector<vector<double>> limits, vector<vector<double>> seeds)
		{
			_rResults.clear();

			_dim = args.size();
			_limits = limits;
			_ranges = vector<double>(_dim);
			for (int i = 0; i < _dim; ++i)
			{
				_ranges[i] = _limits[i][1] - _limits[i][0];
			}
			equation = equation->aggregateConstants();

			_term = AutoDiff::TermUtils::compile(equation, args);

			_rpropStepWidth = vector<double>(_dim);
			_rpropStepConvergenceThreshold = vector<double>(_dim);
			if (seeds.size() > 0)
			{
				for (int i = 0; i < seeds.size(); ++i)
				{
					shared_ptr<RpropResult> r = rPropLoop(seeds[i]);
					_rResults.push_back(r);
					if (r->_finalUtil > 0.75)
					{
						return true;
					}
				}
			}
			int runs = 2 * _dim - seeds.size();
			for (int i = 0; i < runs; ++i)
			{
				shared_ptr<RpropResult> r = rPropLoop(vector<double>());
				_rResults.push_back(r);
				if (r->_finalUtil > 0.75)
				{
					return true;
				}
			}
			int adit = 0;
			while (!evalResults() && adit++ < 20)
			{
				shared_ptr<RpropResult> r = rPropLoop(vector<double>());
				_rResults.push_back(r);
				if (r->_finalUtil > 0.75)
				{
					return true;
				}
			}
			if (adit > 20)
			{
				cerr << "Failed to satisfy heuristic!" << endl;
			}

			return false;
		}

		vector<double> GSolver::solveTest(shared_ptr<AutoDiff::Term> equation,
											vector<shared_ptr<AutoDiff::Variable>> args, vector<vector<double>> limits)
		{
#ifdef GSOLVER_LOG
			initLog();
#endif

			_rResults.clear();
			vector<double> res;

			_dim = args.size();
			_limits = limits;
			_ranges = vector<double>(_dim);
			for (int i = 0; i < _dim; ++i)
			{
				_ranges[i] = _limits[i][1] - _limits[i][0];
			}

			_term = AutoDiff::TermUtils::compile(equation, args);

			_rpropStepWidth = vector<double>(_dim);
			_rpropStepConvergenceThreshold = vector<double>(_dim);
			_runs = 0;
			_fevals = 0;
			//int runs = 1000000;
			while (true)
			{
				_runs++;
				//shared_ptr<RpropResult> r = rPropLoopSimple(vector<double>());
				shared_ptr<RpropResult> r = rPropLoop(vector<double>(), false);
				if (r->_finalUtil > 0.75)
				{
					res = r->_finalValue;
					break;
				}
			}

#ifdef GSOLVER_LOG
			closeLog();
#endif

			return res;
		}

		vector<double> GSolver::solveTest(shared_ptr<AutoDiff::Term> equation,
											vector<shared_ptr<AutoDiff::Variable>> args, vector<vector<double>> limits,
											int maxRuns, bool *found)
		{
#ifdef GSOLVER_LOG
			initLog();
#endif
			*found = false;
			_rResults.clear();
			vector<double> res;

			_dim = args.size();
			_limits = limits;
			_ranges = vector<double>(_dim);
			for (int i = 0; i < _dim; ++i)
			{
				_ranges[i] = _limits[i][1] - _limits[i][0];
			}

			_term = AutoDiff::TermUtils::compile(equation, args);

			_rpropStepWidth = vector<double>(_dim);
			_rpropStepConvergenceThreshold = vector<double>(_dim);
			_runs = 0;
			_fevals = 0;

			while (_runs < maxRuns)
			{
				_runs++;
				//shared_ptr<RpropResult> r = rPropLoopSimple(vector<double>());
				shared_ptr<RpropResult> r = rPropLoop(vector<double>(), false);
				if (r->_finalUtil > 0.75)
				{
					res = r->_finalValue;
					*found = true;
					break;
				}
			}

#ifdef GSOLVER_LOG
			closeLog();
#endif

			return res;
		}

		long GSolver::getRuns()
		{
			return _runs;
		}

		void GSolver::setRuns(long runs)
		{
			_runs = runs;
		}

		long GSolver::getFEvals()
		{
			return _fevals;
		}

		void GSolver::setFEvals(long fevals)
		{
			_fevals = fevals;
		}

		long GSolver::getMaxFEvals()
		{
			return _maxfevals;
		}

		void GSolver::setMaxFEvals(long maxfevals)
		{
			_maxfevals = maxfevals;
		}

		double GSolver::getRPropConvergenceStepSize()
		{
			return _rPropConvergenceStepSize;
		}

		void GSolver::setRPropConvergenceStepSize(double rPropConvergenceStepSize)
		{
			_rPropConvergenceStepSize = rPropConvergenceStepSize;
		}

		vector<double> GSolver::initialPointFromSeed(shared_ptr<RpropResult> res, vector<double> seed)
		{
			pair<vector<double>, double> tup;

			res->_initialValue = vector<double>(_dim);
			res->_finalValue = vector<double>(_dim);
			for (int i = 0; i < _dim; ++i)
			{
				if (std::isnan(seed[i]))
				{
					res->_initialValue[i] = rand() * _ranges[i] + _limits[i][0];
				}
				else
				{
					res->_initialValue[i] = min(max(seed[i], _limits[i][0]), _limits[i][1]);
				}
			}
			tup = _term->differentiate(res->_initialValue);
			res->_initialUtil = tup.second;
			res->_finalUtil = tup.second;

			res->_finalValue.swap(res->_initialValue);

			return tup.first;
		}

		vector<double> GSolver::initialPoint(shared_ptr<RpropResult> res)
		{
			pair<vector<double>, double> tup;
			bool found = true;
			res->_initialValue = vector<double>(_dim);
			res->_finalValue = vector<double>(_dim);
			do
			{
				for (int i = 0; i < _dim; ++i)
				{
					res->_initialValue[i] = rand() * _ranges[i] + _limits[i][0];
				}
				_fevals++;
				tup = _term->differentiate(res->_initialValue);
				for (int i = 0; i < _dim; ++i)
				{
					if (std::isnan(tup.first[i]))
					{
						found = false;
						break;
					}
					else
					{
						found = true;
					}
				}
			} while (!found);
			res->_initialUtil = tup.second;
			res->_finalUtil = tup.second;

			res->_finalValue.swap(res->_initialValue);

			return tup.first;
		}

		shared_ptr<GSolver::RpropResult> GSolver::rPropLoop(vector<double> seed)
		{
			return rPropLoop(seed, false);
		}

		shared_ptr<GSolver::RpropResult> GSolver::rPropLoop(vector<double> seed, bool precise)
		{
			initialStepSize();

			vector<double> curGradient;

			shared_ptr<RpropResult> ret = make_shared<RpropResult>();

			if (seed.size() > 0)
			{
				curGradient = initialPointFromSeed(ret, seed);
			}
			else
			{
				curGradient = initialPoint(ret);
			}
			double curUtil = ret->_initialUtil;

			vector<double> formerGradient = vector<double>(_dim);
			vector<double> curValue = vector<double>(_dim);

			pair<vector<double>, double> tup;

			curValue.swap(ret->_initialValue);

			formerGradient = curGradient;

			int itcounter = 0;
			int badcounter = 0;

#ifdef GSOLVER_LOG
			log(curUtil, curValue);
#endif

			int maxIter = 60;
			int maxBad = 30;
			double minStep = 1E-11;
			if (precise)
			{
				maxIter = 120; //110
				maxBad = 60; //60
				minStep = 1E-15; //15
			}
			int convergendDims = 0;

			while (itcounter++ < maxIter && badcounter < maxBad)
			{
				convergendDims = 0;
				for (int i = 0; i < _dim; ++i)
				{
					if (curGradient[i] * formerGradient[i] > 0)
						_rpropStepWidth[i] *= 1.3;
					else if (curGradient[i] * formerGradient[i] < 0)
						_rpropStepWidth[i] *= 0.5;
					_rpropStepWidth[i] = max(minStep, _rpropStepWidth[i]);
					if (curGradient[i] > 0)
						curValue[i] += _rpropStepWidth[i];
					else if (curGradient[i] < 0)
						curValue[i] -= _rpropStepWidth[i];

					if (curValue[i] > _limits[i][1])
						curValue[i] = _limits[i][1];
					else if (curValue[i] < _limits[i][0])
						curValue[i] = _limits[i][0];
					if (_rpropStepWidth[i] < _rpropStepConvergenceThreshold[i])
						++convergendDims;
				}

				if (!precise && convergendDims >= _dim)
				{
					if (curUtil > ret->_finalUtil)
					{
						ret->_finalUtil = curUtil;
						ret->_finalValue.swap(curValue);
					}

					return ret;
				}

				_fevals++;
				tup = _term->differentiate(curValue);
				bool allZero = true;
				for (int i = 0; i < _dim; ++i)
				{
					if (std::isnan(tup.first[i]))
					{
						ret->_aborted = true;
#ifdef GSOLVER_LOG
						logStep();
#endif
						return ret;
					}
					allZero &= (tup.first[i] == 0);
				}

				curUtil = tup.second;
				formerGradient = curGradient;
				curGradient = tup.first;
#ifdef GSOLVER_LOG
				log(curUtil, curValue);
#endif

				if (curUtil > ret->_finalUtil)
				{
					badcounter = 0;

					ret->_finalUtil = curUtil;
					ret->_finalValue.swap(curValue);

#ifdef ALWAYS_CHECK_THRESHOLD
					if (curUtil > _utilityThreshold)
					{
						return ret;
					}
#endif
				}
				else
				{
					badcounter++;
				}
				if (allZero)
				{
					ret->_aborted = false;
#ifdef GSOLVER_LOG
					logStep();
#endif
					return ret;
				}
			}
#ifdef GSOLVER_LOG
			logStep();
#endif
			ret->_aborted = false;
			return ret;
		}

		shared_ptr<GSolver::RpropResult> GSolver::rPropLoopSimple(vector<double> seed)
		{
			initialStepSize();
			vector<double> curGradient;

			shared_ptr<RpropResult> ret = make_shared<RpropResult>();

			if (seed.size() > 0)
			{
				curGradient = initialPointFromSeed(ret, seed);
			}
			else
			{
				curGradient = initialPoint(ret);
			}
			double curUtil = ret->_initialUtil;

			if (ret->_initialUtil > 0.75)
			{
				return ret;
			}

			vector<double> formerGradient = vector<double>(_dim);
			vector<double> curValue = vector<double>(_dim);

			pair<vector<double>, double> tup;

			curValue.swap(ret->_initialValue);

			formerGradient = curGradient;

			int itcounter = 0;
			int badcounter = 0;

			while (itcounter++ < 40 && badcounter < 6)
			{
				for (int i = 0; i < _dim; ++i)
				{
					if (curGradient[i] * formerGradient[i] > 0)
						_rpropStepWidth[i] *= 1.3;
					else if (curGradient[i] * formerGradient[i] < 0)
						_rpropStepWidth[i] *= 0.5;
					_rpropStepWidth[i] = max(0.0001, _rpropStepWidth[i]);
					if (curGradient[i] > 0)
						curValue[i] += _rpropStepWidth[i];
					else if (curGradient[i] < 0)
						curValue[i] -= _rpropStepWidth[i];

					if (curValue[i] > _limits[i][1])
						curValue[i] = _limits[i][1];
					else if (curValue[i] < _limits[i][0])
						curValue[i] = _limits[i][0];
				}

				tup = _term->differentiate(curValue);
				bool allZero = true;

				for (int i = 0; i < _dim; ++i)
				{
					if (std::isnan(tup.first[i]))
					{
						ret->_aborted = false;
						return ret;
					}
					allZero &= (tup.first[i] == 0);
				}

				curUtil = tup.second;
				formerGradient = curGradient;
				curGradient = tup.first;

				if (curUtil > ret->_finalUtil)
				{
					badcounter = 0;

					ret->_finalUtil = curUtil;
					ret->_finalValue.swap(curValue);

					if (curUtil > 0.75)
					{
						return ret;
					}
				}
				else
				{
					badcounter++;
				}
				if (allZero)
				{
					ret->_aborted = false;
					return ret;
				}
			}

			ret->_aborted = false;
			return ret;
		}

		void GSolver::initialStepSize()
		{
			for (int i = 0; i < _dim; ++i)
			{
				_rpropStepWidth[i] = _initialStepSize * _ranges[i];
				_rpropStepConvergenceThreshold[i] = _rpropStepWidth[i] * _rPropConvergenceStepSize;
			}
		}

		bool GSolver::evalResults()
		{
			/*
			 * TODO: Compare Average Distance between start and end point to
			 * Average distance between any two arbitrary points.
			 *
			 * Latter one can be found in http://www.math.uni-muenster.de/reine/u/burgstal/d18.pdf :
			 * maxDist = sqrt(dim)
			 * avgDist <= 1 / sqrt(6) * sqrt((1+2*sqrt(1-3/(5*dim)))/3) * maxDist
			 *
			 * aprox: (http://www.jstor.org/pss/1427094)
			 *
			 * avgDist = sqrt(dim/3) * (1 - 1/(10*k) - 13/(280*k^2) - 101/ (2800*k^3) - 37533 / (1232000 k^4) * O(sqrt(k)) (?)
			 * */
			int count = _rResults.size();

			int abortedCount = 0;

			vector<double> midInitValue = vector<double>(_dim);

			vector<double> valueInitDev = vector<double>(_dim);

			for (int i = 0; i < count; ++i)
			{
				if (_rResults[i]->_aborted)
				{
					abortedCount++;
				}
				else
				{
					for (int j = 0; j < _dim; ++j)
					{
						midInitValue[j] += _rResults[i]->_initialValue[j];
					}
				}
			}
			if (count - abortedCount < _dim)
			{
				return false;
			}
			for (int j = 0; j < _dim; ++j)
			{
				midInitValue[j] /= count - abortedCount;
			}
			for (int i = 0; i < count; ++i)
			{
				if (_rResults[i]->_aborted)
				{
					continue;
				}
				for (int j = 0; j < _dim; ++j)
				{
					valueInitDev[j] += pow((_rResults[i]->_initialValue[j] - midInitValue[j]) / _ranges[j], 2);
				}
			}
			for (int j = 0; j < _dim; ++j)
			{
				valueInitDev[j] /= count - abortedCount;
				if (valueInitDev[j] < 0.0441)
				{
					return false;
				}
			}

			return true;
		}

		int GSolver::RpropResult::compareTo(shared_ptr<RpropResult> other)
		{
			return _finalUtil > other->_finalUtil ? -1 : 1;
		}

		double GSolver::RpropResult::distanceTraveled()
		{
			double ret = 0;
			for (int i = 0; i < _initialValue.size(); ++i)
			{
				ret += (_initialValue[i] - _finalValue[i]) * (_initialValue[i] - _finalValue[i]);
			}
			return sqrt(ret);
		}

		double GSolver::RpropResult::distanceTraveledNormed(vector<double> ranges)
		{
			double ret = 0;
			for (int i = 0; i < _initialValue.size(); ++i)
			{
				ret += (_initialValue[i] - _finalValue[i]) * (_initialValue[i] - _finalValue[i])
						/ (ranges[i] * ranges[i]);
			}
			return sqrt(ret);
		}

//		bool GSolver::RpropResult::operator<(const shared_ptr<GSolver::RpropResult>& left, const shared_ptr<GSolver::RpropResult>& right)
//		{
//			return left->_finalUtil < right->_finalUtil;
//		}
//
//		bool GSolver::RpropResult::operator>(const shared_ptr<GSolver::RpropResult>& left, const shared_ptr<GSolver::RpropResult>& right)
//		{
//			return left->_finalUtil > right->_finalUtil;
//		}
	} /* namespace Reasoner */
} /* namespace Alica */
