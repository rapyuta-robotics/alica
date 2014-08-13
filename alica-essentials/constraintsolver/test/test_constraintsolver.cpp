#include <gtest/gtest.h>

#include "AutoDiff.h"
#include "GSolver.h"

#include <ctime>
#include <iostream>

using namespace Alica::Reasoner;
using namespace AutoDiff;
using namespace std;

TEST(AutoDiffTest, GSOLVER)
{
	// 12000 * 18000
	const double FIELDLENGTH = 18000;
	const double FIELDWIDTH = 12000;

	shared_ptr<GSolver> g = make_shared<GSolver>();

	Term::setAnd(AndType::AND);
	Term::setOr(OrType::MAX);

	shared_ptr<TVec> ball = make_shared<TVec>(-3000, -2000);

	shared_ptr<Term> equation;
	int posCount = 30;
	int oppCount = 3;
	vector<shared_ptr<Variable>> vars = vector<shared_ptr<Variable>>(posCount * 2);
	vector<vector<double>> limits = vector<vector<double>>(posCount * 2);
	for (int i = 0; i < posCount * 2; ++i)
		limits[i] = vector<double>(2);

	shared_ptr<Term> constraint = LTConstraint::TRUE;
	vector<shared_ptr<TVec>> poses;
	for (int i = 0; i < posCount; ++i)
	{
		vars[i * 2] = make_shared<Variable>();
		vars[i * 2 + 1] = make_shared<Variable>();
		vector<shared_ptr<Term>> tmpVars = vector<shared_ptr<Term>>(2);
		tmpVars = {vars[i * 2], vars[i * 2 + 1]};
		poses.push_back(make_shared<TVec>(tmpVars));

//		constraint &= MSLConstraintBuilder.OutsideArea(Areas.OwnPenaltyArea,poses[i]);
//		constraint &= MSLConstraintBuilder.OutsideArea(Areas.OppPenaltyArea,poses[i]);
//		constraint &= Al.ConstraintBuilder.Distance(ball,poses[i]) > 2000;

		limits[i * 2][0] = -FIELDLENGTH / 2;
		limits[i * 2][1] = FIELDLENGTH / 2;
		limits[i * 2 + 1][0] = -FIELDWIDTH / 2;
		limits[i * 2 + 1][1] = FIELDLENGTH / 2;
	}

	vector<shared_ptr<TVec>> opps;
	opps.push_back(make_shared<TVec>(5000, 0));
	opps.push_back(make_shared<TVec>(500, -2000));
	opps.push_back(make_shared<TVec>(2000, 3300));
	// XXX: random pos

	double util = 0;
//	shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint, 1);
	shared_ptr<ConstraintUtility> csu = make_shared<ConstraintUtility>(constraint, TermBuilder::constant(1));

	//cout << constraint << endl;
	long gt = time(NULL);

	vector<double> res;
	int gsolved = 0;
	int count = 1;
	for (int i = 0; i < count; ++i)
	{
		res = g->solve(csu, vars, limits, &util);
		cout << ".";
		if (util > 0.5)
			gsolved++;
	}
	gt = time(NULL) - gt;

	cout << "GSolver Took " << (gt / 10000.0) << " ms (avg: " << (gt / (10000.0 * count)) << ")" << endl;
	cout << "GSolver Solved: " << gsolved << " times" << endl;
	cout << "Result:" << res[0] << " " << res[1] << " with Utility " << util << endl;
	double actual = TermUtils::evaluate(csu, vars, res);
	cout << "Result:" << actual << endl;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

