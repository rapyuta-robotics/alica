#include "AutoDiff.h"

#include <gtest/gtest.h>

#include <iostream>

using namespace AutoDiff;
using namespace std;

TEST(AutoDiffTest, sin)
{
	// we will use a function of two variables
	shared_ptr<Variable> x = make_shared<Variable>();

	// Define our function: func(x, y) = (x + y) * exp(x - y)
	shared_ptr<Term> func = make_shared<Sin>(x);

	// define the ordering of variables, and a point where
	// the function will be evaluated/differentiated
	vector<shared_ptr<Variable>> vars {x};
	vector<double> point {1};

	// create a compiled version of func.
	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, vars);

	// calculate value and gradient at (x, y) = (1, -2)
	pair<vector<double>, double> gradientAndValue = compiledFunc->differentiate(point);

	double eval = gradientAndValue.second;
	vector<double> gradient = gradientAndValue.first;

	ASSERT_NEAR(0.017452406, eval, 10E-10);
	ASSERT_NEAR(0.999847695, gradient[0], 0.0000001);
}

TEST(AutoDiffTest, compiled)
{
	// we will use a function of two variables
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	// Define our function: func(x, y) = (x + y) * exp(x - y)
	shared_ptr<Term> func = (x + y) * TermBuilder::exp(x - y);

	// define the ordering of variables, and a point where
	// the function will be evaluated/differentiated
	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {1, -2};

	// create a compiled version of func.
	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, vars);

	// calculate value and gradient at (x, y) = (1, -2)
	pair<vector<double>, double> gradientAndValue = compiledFunc->differentiate(point);

	double eval = gradientAndValue.second;
	vector<double> gradient = gradientAndValue.first;

	ASSERT_NEAR(-20.0855369231877, eval, 10E-10);
	ASSERT_NEAR(0, gradient[0], 0.0000001);
	ASSERT_NEAR(40.1710738463753, gradient[1], 10E-10);
}

TEST(AutoDiffTest, uncompiled)
{
	// we will use a function of two variables
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	// Define our function: func(x, y) = (x + y) * exp(x - y)
	shared_ptr<Term> func = (x + y) * TermBuilder::exp(x - y);

	// define the ordering of variables, and a point where
	// the function will be evaluated/differentiated
	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {1, -2};

	// calculate the value and the gradient at the point (x, y) = (1, -2)
	double eval = TermUtils::evaluate(func, vars, point);
	vector<double> gradient = TermUtils::differentiate(func, vars, point);

	ASSERT_NEAR(-20.0855369231877, eval, 10E-10);
	ASSERT_NEAR(0, gradient[0], 0.0000001);
	ASSERT_NEAR(40.1710738463753, gradient[1], 10E-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

