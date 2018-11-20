#include <gtest/gtest.h>

#include "CNSMTGSolver.h"
#include "CNSat.h"
#include "GSolver.h"
#include <autodiff/AutoDiff.h>

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

using namespace std;
using namespace alica;
using namespace alica::reasoner;
using namespace alica::reasoner::cnsat;
using namespace autodiff;

const int minCount = 2, maxCount = 20;
const int minform = 2, maxform = 2;
const long maxTime = 15000; // 000000; //nanos
const int solverCount = 3;
const int count = 1000; // solving count
const double delta = 0.01;

class SolverStats
{
  public:
    int solved = 0;
    double n = 0;
    double mean = 0;
    double m2 = 0;
    int toSlow = 0;

    SolverStats()
    {
        solved = 0;
        n = 0;
        mean = 0;
        m2 = 0;
        toSlow = 0;
    }

    void updateStats(double time, bool solved)
    {
        if (solved)
            this->solved++;

        if (time > maxTime)
            toSlow++;
        else
            toSlow = 0;

        // sw.WriteLine(diff+"\t"+diffSMT+"\t"+util+"\t"+smtutil);
        n += 1.0;
        double delta = time - mean;
        mean += delta / n;
        m2 += delta * (time - mean);
    }

    void reset()
    {
        solved = 0;
        n = 0;
        mean = 0;
        m2 = 0;
    }

    double curVariance() { return (m2 / (n - 1)); }

    string toOutputString()
    {
        stringstream ss;
        ss << mean << "\t" << curVariance() << "\t" << solved;
        return ss.str();
    }
};

// TEST(CNSMTGSolver, SOLVER)
//{
//	vector<shared_ptr<SolverStats>> solverstat = vector<shared_ptr<SolverStats>>(solverCount);
//
//	shared_ptr<GSolver> g = make_shared<GSolver>();
//	shared_ptr<CNSMTGSolver> cnsmt = make_shared<CNSMTGSolver>();
//	cnsmt->setUseIntervalProp(true);
//	shared_ptr<CNSMTGSolver> cnsmtnoIP = make_shared<CNSMTGSolver>();
//	cnsmtnoIP->setUseIntervalProp(false);
//
//	shared_ptr<vector<double>> origBallPoses = make_shared<vector<double>>(10); //{6, -6, 3, -3, 0, -6.5, 6.5};
//	shared_ptr<vector<double>> origBallPosesY = make_shared<vector<double>>(10); //{6, 6, 3, 3, 0, -6.5, -6.5};
//	srand(0);
//
//	for (int formulation = minform; formulation <= maxform; formulation++)
//	{
//		for (int i = 0; i < solverCount; ++i)
//		{
//			solverstat.at(i) = make_shared<SolverStats>();
//		}
//
//		for (int taskCount = minCount; taskCount <= maxCount; taskCount++)
//		{
//			cout << "Formulation: " << formulation << " Taskcount " << taskCount << endl << flush;
//			for (int i = 0; i < solverCount; ++i)
//			{
//				solverstat.at(i)->reset();
//			}
//
//			shared_ptr<vector<double>> ballPoses = make_shared<vector<double>>(taskCount);
//			shared_ptr<vector<double>> ballPosesY = make_shared<vector<double>>(taskCount);
//
//			for (int c = 0; c < count; c++)
//			{
//				for (int i = 0; i < taskCount; i++)
//				{
//					/*ballPoses->at(i) = (((20.0*((double)i))/((double)taskCount))-10.0);
//					 ballPosesY->at(i) = (((20.0*((double)i))/((double)taskCount))-10.0);*/
//					bool isNew;
//					do
//					{
//						isNew = false;
//						ballPoses->at(i) = (20.0 * ((double)rand() / RAND_MAX)) - 10.0;
//						ballPosesY->at(i) = (20.0 * ((double)rand() / RAND_MAX)) - 10.0;
//
//						for (int n = 0; n < i; n++)
//						{
//							//if(Math.Abs(ballPoses->at(n) - ballPoses->at(i)) +
// Math.Abs(ballPosesY->at(n) - ballPosesY->at(i)) < 2*delta) {
//							if ((ballPoses->at(n) - ballPoses->at(i)) * (ballPoses->at(n) -
// ballPoses->at(i))
//									+ (ballPosesY->at(n) - ballPosesY->at(i)) *
//(ballPosesY->at(n)
//-  ballPosesY->at(i)) 									< 4 * delta * delta)
//							{
//								isNew = true;
//								break;
//							}
//						}
//					} while (isNew);
//				}
//
//				int varCount = taskCount;
//				shared_ptr<vector<shared_ptr<Variable>>> variables =
// make_shared<vector<shared_ptr<Variable>>>(varCount*2);
//				shared_ptr<vector<shared_ptr<vector<double>>> > limits =
// make_shared<vector<shared_ptr<vector<double>>>>(varCount*2);
//				shared_ptr<vector<shared_ptr<vector<shared_ptr<Term>>> >> lits =
// make_shared<vector<shared_ptr<vector<shared_ptr<Term>>>>>(varCount*2);
//				for (int i = 0; i < varCount * 2; ++i)
//				{
//					limits->at(i) = make_shared<vector<double>>(2);
//					lits->at(i) = make_shared<vector<shared_ptr<Term>>>(ballPoses->size());
//				}
//				shared_ptr<vector<shared_ptr<Term>>> target =
// make_shared<vector<shared_ptr<Term>>>(varCount);
//				shared_ptr<vector<shared_ptr<Term>>> targetY =
// make_shared<vector<shared_ptr<Term>>>(varCount); 				int agents = 0;
//
//				for (int i = 0; i < varCount * 2; i += 2)
//				{
//					variables->at(i) = make_shared<Variable>();
//					variables->at(i + 1) = make_shared<Variable>();
//				}
//				for (int i = 0; i < varCount * 2; i += 2)
//				{
//					target->at(agents) = variables->at(i);
//					targetY->at(agents) = variables->at(i + 1);
//					limits->at(i)->at(0) = -10.0;
//					limits->at(i)->at(1) = 10.0;
//					limits->at(i + 1)->at(0) = -10.0;
//					limits->at(i + 1)->at(1) = 10.0;
//
//					for (int j = 0; j < ballPoses->size(); j++)
//					{
//						//lits[agents,j] = new AD.Abs(target->at(agents) - ballPoses->at(j)) +
// new  AD.Abs(targetY->at(agents) - ballPosesY->at(j)) < delta;
//						shared_ptr<Term> t1 = target->at(agents) - ballPoses->at(j);
//						shared_ptr<Term> t2 = target->at(agents) - ballPoses->at(j);
//						shared_ptr<Term> t3 = targetY->at(agents) - ballPosesY->at(j);
//						shared_ptr<Term> t4 = targetY->at(agents) - ballPosesY->at(j);
//						lits->at(agents)->at(j) = t1 * t2 + t3 * t4 <
// TermBuilder::constant(delta
//*  delta);
//					}
//					agents++;
//				}
//				// (p1 = X1 u p2 = X2) o (p1 = X2 u p2 = X1)
//				// (p1=X1 o p1=X2) u (p1=X1 o p2=X1) u (p2=X2 o p1=X2) u (p2=X2 o p2=X1)
//				//
//				// (p1=X1 o p1=X2) u (p2=X2 o p2=X1) u
//				// (p1=X1 o p2=X1) u (p2=X2 o p1=X2)
//				//
//				// (p1 = X1 u p2 = X2 u p3 = X3) o (p1 = X2 u p2 = X1 u p3 = X3) o (p1 = X3 u p2 = X2 u
// p3 =  X2) o (p1 = X3 u p2 = X1 u p3 = X2) o (p1 = X1 u p2 = X3 u p3 = X1) o (p1 = X2 u p2 = X3 u p3 = X1)
//				//
//				// (p1 = X1 o p1 = X2 o p1 = X3) u ...
//				// (p1 = X1 o p2 = X1 o p3 = X1) u ...
//
//				shared_ptr<Term> constraint = nullptr;
//
//				{
//					shared_ptr<Term> constr = lits->at(0)->at(0);
//					for (int j = 1; j < ballPoses->size(); j++)
//					{
//						constr = constr | lits->at(0)->at(j);
//					}
//					constraint = constr;
//				}
//
//				for (int i = 1; i < target->size(); i++)
//				{
//					shared_ptr<Term> constr = lits->at(i)->at(0);
//					for (int j = 1; j < ballPoses->size(); j++)
//					{
//						constr = constr | lits->at(i)->at(j);
//					}
//					constraint = constraint & constr;
//				}
//
//				///////////////////////////////////////////////
//				if (formulation == 0)
//				{
//					for (int j = 0; j < ballPoses->size(); j++)
//					{
//						shared_ptr<Term> constr = lits->at(0)->at(j);
//						for (int i = 1; i < target->size(); i++)
//						{
//							constr = constr | lits->at(i)->at(j);
//						}
//						constraint = constraint & constr;
//					}
//				}
//
//				///////////////////////////////////////////////
//				if (formulation == 1)
//				{
//					for (int i = 0; i < target->size(); i++)
//					{
//						for (int j = 0; j < ballPoses->size(); j++)
//						{
//							for (int k = 0; k < i; k++)
//							{
//								constraint = constraint
//										&
// ConstraintBuilder::or_(ConstraintBuilder::not_(lits->at(i)->at(j)),
//																	ConstraintBuilder::not_(lits->at(k)->at(j)));
//							}
//						}
//					}
//				}
//				///////////////////////////////////////////////
//
//				if (formulation == 2)
//				{
//					for (int i = 0; i < target->size(); i++)
//					{
//						for (int k = 0; k < i; k++)
//						{
//							//constraint =  constraint &new AD.Abs(target->at(i) -
// target->at(k))
//+  new
// AD.Abs(targetY->at(i) - targetY->at(k)) > 2*delta; 							constraint =
// constraint
//									& (target->at(i) - target->at(k)) *
//(target->at(i)
//-  target->at(k))
//											+ (targetY->at(i) -
// targetY->at(k))
//* (targetY->at(i) - targetY->at(k))
//											> TermBuilder::constant(4 *
// delta
//*  delta);
//						}
//					}
//				}
//
//				///////////////////////////////////////////////
//				//Console.WriteLine(constraint.ToString());
//				double util = 1;
//				shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint,
//																					TermBuilder::constant(util));
//
//				shared_ptr<vector<double>> res = nullptr;
//				unsigned long long before = 0;
//				unsigned long long diff = 0;
//				util = 0;
//				int curSolver = 0;
//				if (solverstat.at(curSolver)->toSlow < 2)
//				{
//					before = cnsmt->getTime();
//					res = g->solve(csu, variables, limits, &util);
//					diff = (cnsmt->getTime() - before) / 1000000;
//					cout << "x" << flush;
//					if (!(util > 0.5)) {
//						cout << "-" << flush;
//					}
//				}
//				else
//				{
//					diff = maxTime + 1;
//				}
//				solverstat[curSolver]->updateStats(diff, util > 0.5);
//
//				util = 0;
//				curSolver++;
////				if (solverstat.at(curSolver)->toSlow < 2)
////				{
////					before = cnsmt->getTime();
////					res = cnsmt->solve(csu, variables, limits, util);
////					diff = (cnsmt->getTime() - before) / 1000000;
////					cout << "." << flush;
////					if (!(util > 0.5)) {
////						cout << "-" << flush;
////					}
////				}
////				else
////				{
//					diff = maxTime + 1;
////				}
//				solverstat[curSolver]->updateStats(diff, util > 0.5);
//
//				util = 0;
//				curSolver++;
//				if (solverstat.at(curSolver)->toSlow < 2)
//				{
//					before = cnsmtnoIP->getTime();
//					res = cnsmtnoIP->solve(csu, variables, limits, util);
//					diff = (cnsmtnoIP->getTime() - before) / 1000000;
//					cout << "n" << flush;
//					if (!(util > 0.5)) {
//						cout << "-" << flush;
//					}
//				}
//				else
//				{
//					diff = maxTime + 1;
//				}
//				solverstat[curSolver]->updateStats(diff, util > 0.5);
//
//			}
//
//			cout << endl << flush;
//			cout << taskCount << flush;
//			for (int i = 0; i < solverstat.size(); ++i)
//			{
//				cout << "\t" << solverstat.at(i)->toOutputString() << flush;
//			}
//			cout << endl << flush;
//
//		}
//	}
//}
