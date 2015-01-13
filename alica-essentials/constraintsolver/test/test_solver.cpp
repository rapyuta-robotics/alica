#include <gtest/gtest.h>

#include <AutoDiff.h>
#include "CNSat.h"
#include "CNSMTGSolver.h"
#include "GSolver.h"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>

using namespace std;
using namespace alica;
using namespace alica::reasoner;
using namespace alica::reasoner::cnsat;
using namespace autodiff;

TEST(SolverTest, SOLVER)
{
	shared_ptr<GSolver> g = make_shared<GSolver>();
	shared_ptr<CNSMTGSolver> cnsmt = make_shared<CNSMTGSolver>();

	int maxCount = 3;
	shared_ptr<vector<double>> origBallPoses = make_shared<vector<double>>();
	*origBallPoses =
	{	10, -10, 3, -3, 0, -6.5, 6.5};
	shared_ptr<vector<double>> origBallPosesY = make_shared<vector<double>>();
	*origBallPosesY =
	{	10, -10, 3, 3, 0, -6.5, -6.5};

	int formulation = 2;
	int taskCount = 2;
	cout << "Formulation: " << formulation << " Taskcount " << taskCount << endl;

	shared_ptr<vector<double>> ballPoses = make_shared<vector<double>>(taskCount);
	shared_ptr<vector<double>> ballPosesY = make_shared<vector<double>>(taskCount);

	for (int i = 0; i < taskCount; i++)
	{
		ballPoses->at(i) = origBallPoses->at(i);
		ballPosesY->at(i) = origBallPoses->at(i);
		cout << ballPoses->at(i) << " " << ballPosesY->at(i) << endl;
	}

	int varCount = taskCount;
	shared_ptr<vector<shared_ptr<Variable>>> variables = make_shared<vector<shared_ptr<Variable>>>(varCount);
	shared_ptr<vector<shared_ptr<vector<double>>> > limits = make_shared<vector<shared_ptr<vector<double>>>>(varCount);
	for (int i = 0; i < varCount; ++i)
	{
		limits->at(i) = make_shared<vector<double>>(2);
	}
	shared_ptr<vector<shared_ptr<vector<shared_ptr<Term>>> >> lits = make_shared<vector<shared_ptr<vector<shared_ptr<Term>>>>>(varCount*2);
	for (int i = 0; i < varCount * 2; ++i)
	{
		lits->at(i) = make_shared<vector<shared_ptr<Term>>>(ballPoses->size());
	}
	shared_ptr<vector<shared_ptr<Term>>> target = make_shared<vector<shared_ptr<Term>>>(varCount);
	shared_ptr<vector<shared_ptr<Term>>> targetY = make_shared<vector<shared_ptr<Term>>>(varCount);
	int agents = 0;

	double delta = 0.05;

	/*			for(int i=0; i<varCount*2; i+=2) {
	 variables->at(i) = new AutoDiff.Variable();
	 variables[i+1] = new AutoDiff.Variable();
	 target->at(agents) = variables->at(i);
	 targetY->at(agents) = variables[i+1];
	 limits[i,0] = -10.1;
	 limits[i,1] = 10.1;
	 limits[i+1,0] = -10.1;
	 limits[i+1,1] = 10.1;

	 for(int j=0; j<ballPoses->size(); j++) {
	 lits[agents,j] = new AD.Abs(target->at(agents) - ballPoses->at(j)) + new AD.Abs(targetY->at(agents) - ballPosesY->at(j)) < delta;
	 }
	 agents++;
	 }*/

	for (int i = 0; i < varCount; i++)
	{
		variables->at(i) = make_shared<Variable>();
		target->at(agents) = variables->at(i);
		limits->at(i)->at(0) = -10.1;
		limits->at(i)->at(1) = 10.1;

		for (int j = 0; j < ballPoses->size(); j++)
		{
			shared_ptr<Term> left = dynamic_pointer_cast<Term>(make_shared<Abs>(target->at(agents)- ballPoses->at(j)));
			lits->at(agents)->at(j) = left < TermBuilder::constant(delta);
		}
		agents++;
	}

// (p1 = X1 u p2 = X2) o (p1 = X2 u p2 = X1)
// (p1=X1 o p1=X2) u (p1=X1 o p2=X1) u (p2=X2 o p1=X2) u (p2=X2 o p2=X1)
//
// (p1=X1 o p1=X2) u (p2=X2 o p2=X1) u
// (p1=X1 o p2=X1) u (p2=X2 o p1=X2)
//
// (p1 = X1 u p2 = X2 u p3 = X3) o (p1 = X2 u p2 = X1 u p3 = X3) o (p1 = X3 u p2 = X2 u p3 = X2) o (p1 = X3 u p2 = X1 u p3 = X2) o (p1 = X1 u p2 = X3 u p3 = X1) o (p1 = X2 u p2 = X3 u p3 = X1)
//
// (p1 = X1 o p1 = X2 o p1 = X3) u ...
// (p1 = X1 o p2 = X1 o p3 = X1) u ...

	shared_ptr<Term> constraint = nullptr;
	if (formulation == 0)
	{
		{
			shared_ptr<Term> constr = lits->at(0)->at(0);
			for (int j = 1; j < ballPoses->size(); j++)
			{
				constr = constr | lits->at(0)->at(j);
			}
			constraint = constr;
		}

		for (int i = 1; i < target->size(); i++)
		{
			shared_ptr<Term> constr = lits->at(i)->at(0);
			for (int j = 1; j < ballPoses->size(); j++)
			{
				constr = constr | lits->at(i)->at(j);
			}
			constraint = constraint & constr;
		}

		for (int j = 0; j < ballPoses->size(); j++)
		{
			double p = ballPoses->at(j);
			shared_ptr<Term> constr = lits->at(0)->at(j);
			for (int i = 1; i < target->size(); i++)
			{
				constr = constr | lits->at(i)->at(j);
			}
			constraint = constraint & constr;
		}
	}

///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////
	if (formulation >= 1)
	{
		{
			shared_ptr<Term> constr = lits->at(0)->at(0);
			for (int j = 1; j < ballPoses->size(); j++)
			{
				constr = constr | lits->at(0)->at(j);
			}
			constraint = constr;
		}

		for (int i = 1; i < target->size(); i++)
		{
			shared_ptr<Term> constr = lits->at(i)->at(0);
			for (int j = 1; j < ballPoses->size(); j++)
			{
				constr = constr | lits->at(i)->at(j);
			}
			constraint = constraint & constr;
		}
	}
///////////////////////////////////////////////
	if (formulation == 1)
	{
		for (int i = 0; i < target->size(); i++)
		{
			for (int j = 0; j < ballPoses->size(); j++)
			{
				for (int k = 0; k < i; k++)
				{
					constraint = constraint
							& ConstraintBuilder::or_(ConstraintBuilder::not_(lits->at(i)->at(j)),
														ConstraintBuilder::not_(lits->at(k)->at(j)));
				}
			}
		}
	}
///////////////////////////////////////////////
	if (formulation == 2)
	{
		constraint = lits->at(0)->at(0);
		//constraint = constraint & ConstraintBuilder::not_(lits->at(1)->at(0));
		constraint = constraint & lits->at(1)->at(1);
	}

///////////////////////////////////////////////
//cout << constraint->toString() << endl;
	double util = 1;
	shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint, TermBuilder::constant(util));

	shared_ptr<vector<double>> res = g->solve(csu, variables, limits, &util);
	cout << "Solved with Util:" << util << endl;
	for (int i = 0; i < res->size(); i++)
	{
		cout << res->at(i) << "\t";
	}
	cout << endl;

//	cout << csu->toString() << endl; // TODO: implement toString
	cout << "Variable Count: " << variables->size() << endl;

	// Sample
//	shared_ptr<ICompiledTerm> t = TermUtils::compile(csu, variables);
//	for (double x = -10; x <= 10; x += 0.25)
//	{
//		for (double y = -10; y <= 10; y += 0.25)
//		{
//			auto asd = make_shared<vector<double>>();
//			*asd =
//			{	x, y};
//			double val = t->differentiate(asd).second;
//			cout << x << "\t" << y << "\t" << val << endl;
//		}
//		cout << endl;
//	}
}
