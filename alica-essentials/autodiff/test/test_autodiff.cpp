#include "AutoDiff.h"
#include <gtest/gtest.h>

#include <iostream>

using namespace AutoDiff;
using namespace std;

// Declare a test
TEST(AutoDiffBasics, readValues)
{
	// we will use a function of two variables
	Variable x;
	Variable y;

	// Define our function: func(x, y) = (x + y) * exp(x - y)
	Term func = (x + y) * TermBuilder::exp(x - y);

	// define the ordering of variables, and a point where
	// the function will be evaluated/differentiated
//	Variable[] vars = { x, y };
//	double[] point = { 1, -2 };

	// calculate the value and the gradient at the point (x, y) = (1, -2)
//	double eval = func.evaluate(vars, point);
//	double[] gradient = func.differentiate(vars, point);


/*
	// create a compiled version of func.
	ICompiledTerm compiledFunc = func.compile(x, y);

	// calculate value and gradient at (x, y) = (1, -2)
	pair<double[], double> gradientAndValue =
		compiledFunc.differentiate(1, -2);

	double eval = gradientAndValue.item2;
	double[] gradient = gradientAndValue.item1;*/
/*
	EXPECT_EQ(-20.0855369231877, eval);
	EXPECT_EQ(0, gradient[0]);
	EXPECT_EQ(40.1710738463753, gradient[1]);*/

	// determine the path to the test config
//	string path = FileSystem::getSelfPath();

//	int place = path.rfind("devel");
//	path = path.substr(0, place);
//	path = path + "src/supplementary/system_config/test";
	// bring up the SystemConfig with the corresponding path
//	SystemConfig* sc = SystemConfig::getInstance();
//	sc->setRootPath(path);
//	sc->setConfigPath(path + "/etc");

	// read int
//	int intTestValue = (*sc)["Test"]->get<int>("intTestValue", NULL);
//	EXPECT_EQ(221, intTestValue);
//
//	// read double
//	double doubleTestValue = (*sc)["Test"]->get<double>("doubleTestValue", NULL);
//	EXPECT_DOUBLE_EQ(0.66234023823, doubleTestValue);
//
//	// read float
//	float floatTestValue = (*sc)["Test"]->get<float>("floatTestValue", NULL);
//	EXPECT_FLOAT_EQ(1.14f, floatTestValue);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

