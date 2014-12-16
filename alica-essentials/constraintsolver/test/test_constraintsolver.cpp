#include <gtest/gtest.h>

#include "AutoDiff.h"
#include "GSolver.h"
#include "MSLConstraintBuilder.h"
#include "ConstraintBuilder.h"
#include <clock/AlicaROSClock.h>

#include "CNSMTGSolver.h"

#include <ctime>
#include <iostream>

#include <cmath>

using namespace std;
using namespace autodiff;
using namespace alica;
using namespace alica::reasoner;
using namespace carpenoctem::base;

TEST(AutoDiffTest, GSOLVER)
{
	// 12000 * 18000
	const double FIELDLENGTH = 18000;
	const double FIELDWIDTH = 12000;

	shared_ptr<GSolver> g = make_shared<GSolver>();

	Term::setAnd(AndType::AND);
	Term::setOr(OrType::MAX);

	shared_ptr<TVec> ball = make_shared<TVec>(initializer_list<double>{-3000, -2000});

	shared_ptr<Term> equation;
	int posCount = 30;
	int oppCount = 3;
	auto vars = make_shared<vector<shared_ptr<Variable>>>(posCount * 2);
	auto limits = make_shared<vector<shared_ptr<vector<double>>>>(posCount * 2);
	for (int i = 0; i < posCount * 2; ++i)
		limits->at(i) = make_shared<vector<double>>(2);

	shared_ptr<Term> constraint = LTConstraint::TRUE;
	vector<shared_ptr<TVec>> poses;
	for (int i = 0; i < posCount; ++i)
	{
		vars->at(i * 2) = make_shared<Variable>();
		vars->at(i * 2 + 1) = make_shared<Variable>();
		poses.push_back(make_shared<TVec>(initializer_list<shared_ptr<Term>>{vars->at(i * 2), vars->at(i * 2 + 1)}));

		constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OwnPenaltyArea, poses[i]);
		constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OppPenaltyArea, poses[i]);
		constraint = constraint & (ConstraintBuilder::distance(ball, poses[i]) > TermBuilder::constant(2000));

		limits->at(i * 2)->at(0) = -FIELDLENGTH / 2;
		limits->at(i * 2)->at(1) = FIELDLENGTH / 2;
		limits->at(i * 2 + 1)->at(0) = -FIELDWIDTH / 2;
		limits->at(i * 2 + 1)->at(1) = FIELDWIDTH / 2;
	}

	vector<shared_ptr<TVec>> opps;
	opps.push_back(make_shared<TVec>(initializer_list<double>{5000, 0}));
	opps.push_back(make_shared<TVec>(initializer_list<double>{500, -2000}));
	opps.push_back(make_shared<TVec>(initializer_list<double>{2000, 3300}));
	// XXX: random pos

	double util = 0;
	shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint, TermBuilder::constant(1));


	//cout << constraint << endl;
	long gt = time(NULL);

	shared_ptr<vector<double>> res;
	int gsolved = 0;
	int count = 1;
	for (int i = 0; i < count; ++i)
	{
		res = g->solve(csu, vars, limits, &util);
		cout << ".";
		if (util > 0.5)
			gsolved++;
	}
	cout << endl;
	gt = time(NULL) - gt;

	cout << "GSolver Took " << (gt / 10000.0) << " ms (avg: " << (gt / (10000.0 * count)) << ")" << endl;
	cout << "GSolver Solved: " << gsolved << " times" << endl;
	cout << "Result:" << res->at(0) << " " << res->at(1) << " with Utility " << util << endl;
	double actual = TermUtils::evaluate(csu, vars, res);
	cout << "Result:" << actual << endl;
}

TEST(AutoDiffTest, GSOLVER_UTIL)
{
	// 12000 * 18000
	const double FIELDLENGTH = 18000;
	const double FIELDWIDTH = 12000;

	shared_ptr<GSolver> g = make_shared<GSolver>();

	Term::setAnd(AndType::AND);
	Term::setOr(OrType::MAX);

	shared_ptr<TVec> ball = make_shared<TVec>(initializer_list<double>{-3000, -2000});

	shared_ptr<Term> equation;
	int posCount = 30;
	int oppCount = 3;
	auto vars = make_shared<vector<shared_ptr<Variable>>>(posCount * 2);
	auto limits = make_shared<vector<shared_ptr<vector<double>>>>(posCount * 2);
	for (int i = 0; i < posCount * 2; ++i)
		limits->at(i) = make_shared<vector<double>>(2);

	shared_ptr<Term> constraint = LTConstraint::TRUE;
	vector<shared_ptr<TVec>> poses;
	for (int i = 0; i < posCount; ++i)
	{
		vars->at(i * 2) = make_shared<Variable>();
		vars->at(i * 2 + 1) = make_shared<Variable>();
		poses.push_back(make_shared<TVec>(initializer_list<shared_ptr<Term>>{vars->at(i * 2), vars->at(i * 2 + 1)}));

		constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OwnPenaltyArea, poses[i]);
		constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OppPenaltyArea, poses[i]);
		constraint = constraint & (ConstraintBuilder::distance(ball, poses[i]) > TermBuilder::constant(2000));

		limits->at(i * 2)->at(0) = -FIELDLENGTH / 2;
		limits->at(i * 2)->at(1) = FIELDLENGTH / 2;
		limits->at(i * 2 + 1)->at(0) = -FIELDWIDTH / 2;
		limits->at(i * 2 + 1)->at(1) = FIELDWIDTH / 2;
	}

	vector<shared_ptr<TVec>> opps;
	opps.push_back(make_shared<TVec>(initializer_list<double>{5000, 0}));
	opps.push_back(make_shared<TVec>(initializer_list<double>{500, -2000}));
	opps.push_back(make_shared<TVec>(initializer_list<double>{2000, 3300}));
	// XXX: random pos

	shared_ptr<Term> utilCsu = TermBuilder::constant(0);
	for (int i = 0; i < posCount; ++i) {
//		TVec ownpos = new TVec(vars[0], vars[1]);
//	    Term util = 2*FIELDLENGTH - ConstraintBuilder::distance(ball, ownpos);
		shared_ptr<TVec> ownpos = make_shared<TVec>(initializer_list<shared_ptr<Term>>{vars->at(0), vars->at(1)});
		utilCsu = utilCsu + TermBuilder::constant(2*FIELDLENGTH) - ConstraintBuilder::distance(ball, ownpos);
	}

	double util = 0;
	shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint, utilCsu);


	//cout << constraint << endl;
	long gt = time(NULL);

	shared_ptr<vector<double>> res;
	int gsolved = 0;
	int count = 1;
	for (int i = 0; i < count; ++i)
	{
		res = g->solve(csu, vars, limits, &util);
		cout << ".";
		if (util > 0.5)
			gsolved++;
	}
	cout << endl;
	gt = time(NULL) - gt;

	cout << "GSolver Took " << (gt / 10000.0) << " ms (avg: " << (gt / (10000.0 * count)) << ")" << endl;
	cout << "GSolver Solved: " << gsolved << " times" << endl;
	cout << "Result:" << res->at(0) << " " << res->at(1) << " with Utility " << util << endl;
	double actual = TermUtils::evaluate(csu, vars, res);
	cout << "Result:" << actual << endl;
}

TEST(AutoDiffTest, CNSMTGSOLVER)
{
	// 12000 * 18000
	const double FIELDLENGTH = 18000;
	const double FIELDWIDTH = 12000;

	shared_ptr<CNSMTGSolver> g = make_shared<CNSMTGSolver>();

	Term::setAnd(AndType::AND);
	Term::setOr(OrType::MAX);

	shared_ptr<TVec> ball = make_shared<TVec>(initializer_list<double>{-3000, -2000});

	shared_ptr<Term> equation;
	int posCount = 30;
	int oppCount = 3;
	auto vars = make_shared<vector<shared_ptr<Variable>>>(posCount * 2);
	auto limits = make_shared<vector<shared_ptr<vector<double>>>>(posCount * 2);
	for (int i = 0; i < posCount * 2; ++i)
		limits->at(i) = make_shared<vector<double>>(2);

	shared_ptr<Term> constraint = LTConstraint::TRUE;
	vector<shared_ptr<TVec>> poses;
	for (int i = 0; i < posCount; ++i)
	{
		vars->at(i * 2) = make_shared<Variable>();
		vars->at(i * 2 + 1) = make_shared<Variable>();
		poses.push_back(make_shared<TVec>(initializer_list<shared_ptr<Term>>{vars->at(i * 2), vars->at(i * 2 + 1)}));

		constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OwnPenaltyArea, poses[i]);
		constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OppPenaltyArea, poses[i]);
		constraint = constraint & (ConstraintBuilder::distance(ball, poses[i]) > TermBuilder::constant(2000));

		limits->at(i * 2)->at(0) = -FIELDLENGTH / 2;
		limits->at(i * 2)->at(1) = FIELDLENGTH / 2;
		limits->at(i * 2 + 1)->at(0) = -FIELDWIDTH / 2;
		limits->at(i * 2 + 1)->at(1) = FIELDWIDTH / 2;
	}

	vector<shared_ptr<TVec>> opps;
	opps.push_back(make_shared<TVec>(initializer_list<double>{5000, 0}));
	opps.push_back(make_shared<TVec>(initializer_list<double>{500, -2000}));
	opps.push_back(make_shared<TVec>(initializer_list<double>{2000, 3300}));
	// XXX: random pos

	double util = 0;
	shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint, TermBuilder::constant(1));


	//cout << constraint << endl;
	long gt = time(NULL);

	shared_ptr<vector<double>> res;
	int gsolved = 0;
	int count = 1;
	for (int i = 0; i < count; ++i)
	{
		res = g->solve(csu, vars, limits, &util);
		cout << ".";
		if (util > 0.5)
			gsolved++;
	}
	cout << endl;
	gt = time(NULL) - gt;

	cout << "CNSMTGSolver Took " << (gt / 10000.0) << " ms (avg: " << (gt / (10000.0 * count)) << ")" << endl;
	cout << "CNSMTGSolver Solved: " << gsolved << " times" << endl;
	cout << "Result:" << res->at(0) << " " << res->at(1) << " with Utility " << util << endl;
	double actual = TermUtils::evaluate(csu, vars, res);
	cout << "Result:" << actual << endl;
}

TEST(AutoDiffTest, CNSMTGSOLVER_UTIL)
{
	// 12000 * 18000
	const double FIELDLENGTH = 18000;
	const double FIELDWIDTH = 12000;

	shared_ptr<CNSMTGSolver> g = make_shared<CNSMTGSolver>();

	Term::setAnd(AndType::AND);
	Term::setOr(OrType::MAX);

	shared_ptr<TVec> ball = make_shared<TVec>(initializer_list<double>{-3000, -2000});

	shared_ptr<Term> equation;
	int posCount = 30;
	int oppCount = 3;
	auto vars = make_shared<vector<shared_ptr<Variable>>>(posCount * 2);
	auto limits = make_shared<vector<shared_ptr<vector<double>>>>(posCount * 2);
	for (int i = 0; i < posCount * 2; ++i)
		limits->at(i) = make_shared<vector<double>>(2);

	shared_ptr<Term> constraint = LTConstraint::TRUE;
	vector<shared_ptr<TVec>> poses;
	for (int i = 0; i < posCount; ++i)
	{
		vars->at(i * 2) = make_shared<Variable>();
		vars->at(i * 2 + 1) = make_shared<Variable>();
		poses.push_back(make_shared<TVec>(initializer_list<shared_ptr<Term>>{vars->at(i * 2), vars->at(i * 2 + 1)}));

		constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OwnPenaltyArea, poses[i]);
		constraint = constraint & MSLConstraintBuilder::outsideArea(Areas::OppPenaltyArea, poses[i]);
		constraint = constraint & (ConstraintBuilder::distance(ball, poses[i]) > TermBuilder::constant(2000));

		limits->at(i * 2)->at(0) = -FIELDLENGTH / 2;
		limits->at(i * 2)->at(1) = FIELDLENGTH / 2;
		limits->at(i * 2 + 1)->at(0) = -FIELDWIDTH / 2;
		limits->at(i * 2 + 1)->at(1) = FIELDWIDTH / 2;
	}

	vector<shared_ptr<TVec>> opps;
	opps.push_back(make_shared<TVec>(initializer_list<double>{5000, 0}));
	opps.push_back(make_shared<TVec>(initializer_list<double>{500, -2000}));
	opps.push_back(make_shared<TVec>(initializer_list<double>{2000, 3300}));
	// XXX: random pos

	shared_ptr<Term> utilCsu = TermBuilder::constant(0);
	for (int i = 0; i < posCount; ++i) {
//		TVec ownpos = new TVec(vars[0], vars[1]);
//	    Term util = 2*FIELDLENGTH - ConstraintBuilder::distance(ball, ownpos);
		shared_ptr<TVec> ownpos = make_shared<TVec>(initializer_list<shared_ptr<Term>>{vars->at(0), vars->at(1)});
		utilCsu = utilCsu + TermBuilder::constant(2*FIELDLENGTH) - ConstraintBuilder::distance(ball, ownpos);
	}

	double util = 0;
	shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint, utilCsu);


	//cout << constraint << endl;
	long gt = time(NULL);

	shared_ptr<vector<double>> res;
	int gsolved = 0;
	int count = 1;
	for (int i = 0; i < count; ++i)
	{
		res = g->solve(csu, vars, limits, &util);
		cout << ".";
		if (util > 0.5)
			gsolved++;
	}
	cout << endl;
	gt = time(NULL) - gt;

	cout << "CNSMTGSolver Took " << (gt / 10000.0) << " ms (avg: " << (gt / (10000.0 * count)) << ")" << endl;
	cout << "CNSMTGSolver Solved: " << gsolved << " times" << endl;
	cout << "Result:" << res->at(0) << " " << res->at(1) << " with Utility " << util << endl;
	double actual = TermUtils::evaluate(csu, vars, res);
	cout << "Result:" << actual << endl;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

