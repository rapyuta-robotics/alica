#include "AutoDiff.h"

#include <gtest/gtest.h>

using namespace autodiff;
using namespace std;

TEST(AutoDiffTest, ABS)
{
	shared_ptr<Variable> x = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Abs>(x);

	vector<shared_ptr<Variable>> vars {x};
	vector<double> point {-13};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(13, eval);
	ASSERT_DOUBLE_EQ(0, gradient->at(0));
}

TEST(AutoDiffTest, AND)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<And>(x, y);

	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {13, 37};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(1, eval);
	ASSERT_DOUBLE_EQ(0, gradient->at(0));
	ASSERT_DOUBLE_EQ(0, gradient->at(1));
}

TEST(AutoDiffTest, ATAN2)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Atan2>(x, y);

	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {13, 37};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_NEAR(0.337878, eval, 10E-6);
	ASSERT_NEAR(-0.024057, gradient->at(0), 10E-6);
	ASSERT_NEAR(0.008453, gradient->at(1), 10E-6);
}

TEST(AutoDiffTest, CONSTPOWER)
{
	shared_ptr<Variable> x = make_shared<Variable>();

	shared_ptr<Term> func = TermBuilder::power(x, 2);

	vector<shared_ptr<Variable>> vars {x};
	vector<double> point {13};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(169, eval);
	ASSERT_DOUBLE_EQ(26, gradient->at(0));
}

TEST(AutoDiffTest, CONSTRAINTUTILITY)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<ConstraintUtility>(x, y);

	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {13, 37};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(37, eval);
	ASSERT_DOUBLE_EQ(0, gradient->at(0));
	ASSERT_DOUBLE_EQ(1, gradient->at(1));
}

TEST(AutoDiffTest, COS)
{
	shared_ptr<Variable> x = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Cos>(x);

	vector<shared_ptr<Variable>> vars {x};
	vector<double> point {13};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_NEAR(0.907447, eval, 10E-6);
	ASSERT_NEAR(-0.420167, gradient->at(0), 10E-6);
}

TEST(AutoDiffTest, EXP)
{
	shared_ptr<Variable> x = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Exp>(x);

	vector<shared_ptr<Variable>> vars {x};
	vector<double> point {13};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_NEAR(442413, eval, 10E-1);
	ASSERT_NEAR(442413, gradient->at(0), 10E-1);
}

TEST(AutoDiffTest, LINSIGMOID)
{
	shared_ptr<Variable> x = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<LinSigmoid>(x);

	vector<shared_ptr<Variable>> vars {x};
	vector<double> point {13};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(0.5, eval);
	ASSERT_DOUBLE_EQ(0, gradient->at(0));
}

TEST(AutoDiffTest, LOG)
{
	shared_ptr<Variable> x = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Log>(x);

	vector<shared_ptr<Variable>> vars {x};
	vector<double> point {13};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_NEAR(2.56495, eval, 10E-5);
	ASSERT_NEAR(0, gradient->at(0), 10E-1);
}

TEST(AutoDiffTest, LTCONSTRAINT)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<LTConstraint>(x, y, 0.01);

	vector<shared_ptr<Variable>> vars {x, y};
	{
		vector<double> point {13, 37};

		shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
		pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
				make_shared<vector<double>>(point));

		double eval = gradientAndValue.second;
		shared_ptr<vector<double>> gradient = gradientAndValue.first;

		ASSERT_DOUBLE_EQ(1, eval);
		ASSERT_DOUBLE_EQ(0, gradient->at(0));
		ASSERT_DOUBLE_EQ(0, gradient->at(1));
	}

	{
		vector<double> point {37, 13};

		shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
		pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
				make_shared<vector<double>>(point));

		double eval = gradientAndValue.second;
		shared_ptr<vector<double>> gradient = gradientAndValue.first;

		ASSERT_NEAR(-0.24, eval, 10E-2);
		ASSERT_DOUBLE_EQ(-0.01, gradient->at(0));
		ASSERT_DOUBLE_EQ(0.01, gradient->at(1));
	}
}

TEST(AutoDiffTest, LTECONSTRAINT)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<LTEConstraint>(x, y, 0.01);

	vector<shared_ptr<Variable>> vars {x, y};
	{
		vector<double> point {13, 37};

		shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
		pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
				make_shared<vector<double>>(point));

		double eval = gradientAndValue.second;
		shared_ptr<vector<double>> gradient = gradientAndValue.first;

		ASSERT_DOUBLE_EQ(1, eval);
		ASSERT_DOUBLE_EQ(0, gradient->at(0));
		ASSERT_DOUBLE_EQ(0, gradient->at(1));
	}

	{
		vector<double> point {37, 13};

		shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
		pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
				make_shared<vector<double>>(point));

		double eval = gradientAndValue.second;
		shared_ptr<vector<double>> gradient = gradientAndValue.first;

		ASSERT_NEAR(-0.24, eval, 10E-2);
		ASSERT_DOUBLE_EQ(-0.01, gradient->at(0));
		ASSERT_DOUBLE_EQ(0.01, gradient->at(1));
	}
}

TEST(AutoDiffTest, MAX)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Max>(x, y);

	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {13, 37};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(37, eval);
	ASSERT_DOUBLE_EQ(0, gradient->at(0));
}

TEST(AutoDiffTest, MIN)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Min>(x, y);

	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {13, 37};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(13, eval);
	ASSERT_DOUBLE_EQ(1, gradient->at(0));
}

TEST(AutoDiffTest, OR)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Or>(x, y);

	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {13, 37};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(1, eval);
	ASSERT_DOUBLE_EQ(0, gradient->at(0));
}

TEST(AutoDiffTest, REIFICATION)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Reification>(x, 5, 25);

	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {13, 37};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(25, eval);
	ASSERT_DOUBLE_EQ(20, gradient->at(0));
}

TEST(AutoDiffTest, SIN)
{
	shared_ptr<Variable> x = make_shared<Variable>();

	shared_ptr<Term> func = make_shared<Sin>(x);

	vector<shared_ptr<Variable>> vars {x};
	vector<double> point {13};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_NEAR(0.420167, eval, 10E-6);
	ASSERT_NEAR(0.907447, gradient->at(0), 10E-6);
}

TEST(AutoDiffTest, TERMPOWER)
{
	shared_ptr<Variable> x = make_shared<Variable>();
	shared_ptr<Variable> y = make_shared<Variable>();

	shared_ptr<Term> func = TermBuilder::power(x, y);

	vector<shared_ptr<Variable>> vars {x, y};
	vector<double> point {13, 2};

	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_DOUBLE_EQ(169, eval);
	ASSERT_DOUBLE_EQ(26, gradient->at(0));
	ASSERT_NEAR(433.476441, gradient->at(1), 10E-6);
}

#include <limits>
#include <memory>

TEST(AutoDiffTest, EQUALITY)
{
	shared_ptr<Term> TRUE = TermBuilder::constant(1);

	ASSERT_TRUE(TRUE == TRUE);
}

TEST(AutoDiffTest, COMPILED)
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
	shared_ptr<ICompiledTerm> compiledFunc = TermUtils::compile(func, make_shared<vector<shared_ptr<Variable>>>(vars));

	// calculate value and gradient at (x, y) = (1, -2)
	pair<shared_ptr<vector<double>>, double> gradientAndValue = compiledFunc->differentiate(
			make_shared<vector<double>>(point));

	double eval = gradientAndValue.second;
	shared_ptr<vector<double>> gradient = gradientAndValue.first;

	ASSERT_NEAR(-20.0855369231877, eval, 10E-10);
	ASSERT_NEAR(0, gradient->at(0), 10E-10);
	ASSERT_NEAR(40.1710738463753, gradient->at(1), 10E-10);
}

TEST(AutoDiffTest, UNCOMPILED)
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
	double eval = TermUtils::evaluate(func, make_shared<vector<shared_ptr<Variable>>>(vars), make_shared<vector<double>>(point));
	shared_ptr<vector<double>> gradient = TermUtils::differentiate(func, make_shared<vector<shared_ptr<Variable>>>(vars), make_shared<vector<double>>(point));

	ASSERT_NEAR(-20.0855369231877, eval, 10E-10);
	ASSERT_NEAR(0, gradient->at(0), 10E-10);
	ASSERT_NEAR(40.1710738463753, gradient->at(1), 10E-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

