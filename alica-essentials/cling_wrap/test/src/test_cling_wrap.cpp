/*
 * test_cling_wrap.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: Stephan Opfer
 */

#include <gtest/gtest.h>
#include "ClingWrapper.h"
#include <chrono>

using namespace std;

TEST(ClingWrap, nqueens)
{
	chrono::_V2::system_clock::time_point start = chrono::high_resolution_clock::now();

	supplementary::ClingWrapper cw;
	cw.addKnowledgeFile("data/queens/queens1.lp");
	cw.init();
	cw.solve();
	cw.printLastModel();
	std::chrono::_V2::system_clock::time_point end = chrono::high_resolution_clock::now();
	cout << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;
}

TEST(ClingWrap, incqueens)
{
	chrono::_V2::system_clock::time_point start = chrono::high_resolution_clock::now();

	supplementary::ClingWrapper cw;
	cw.addKnowledgeFile("data/queens/incqueens.lp");
	cw.init();
//	cw.ground("board", {10});
//	cw.solve();
//	cw.printLastModel();

	std::chrono::_V2::system_clock::time_point end = chrono::high_resolution_clock::now();
	cout << "Init-Time:" << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;

	for (int i = 1; i < 10; i++)
	{
		chrono::_V2::system_clock::time_point incstart = chrono::high_resolution_clock::now();
		cw.ground("board", {i});
		cw.solve();
		cw.printLastModel();
		std::chrono::_V2::system_clock::time_point incend = chrono::high_resolution_clock::now();

//		for(long j=1; j < 10; j++) {
//			cout <<"hmmm";
//		}

// TODO komische ausgaben vom Solver wegbekommen?!
		//sleep(1);
		cout << "Inc-Time:" << chrono::duration_cast<chrono::nanoseconds>(incend - incstart).count() << " ns" << endl;
	}
}

TEST(ClingWrap, iclingo)
{
	chrono::_V2::system_clock::time_point start = chrono::high_resolution_clock::now();

	supplementary::ClingWrapper cw;
	cw.addKnowledgeFile("data/iclingo/example.lp");
	cw.init();
//	cw.ground("board", {10});
//	cw.solve();
//	cw.printLastModel();

	std::chrono::_V2::system_clock::time_point end = chrono::high_resolution_clock::now();
	cout << "Init-Time:" << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;
	int step = 1;
	int iquery = 1;
	int imin = 1;
	int imax = 11;
	string istop = "UNSAT";

	while (true)
	{
		if (step > imax)
		{
			break;
		}
		cw.ground("cumulative", {step});
		if (step >= iquery)
		{
			if (step > iquery)
			{
				cw.releaseExternal("query", {step - 1});
			}
		}
		cw.assignExternal("query", {step}, true);
		cw.ground("volatile", {step});

		Gringo::SolveResult ret = cw.solve();
		cw.printLastModel();
		if (step >= imin && ((istop == "SAT" && ret == Gringo::SolveResult::SAT) || (istop == "UNSAT" && ret != Gringo::SolveResult::SAT)))
		{
			break;
		}
		step = step + 1;
	}
}

TEST(ClingWrap, simpleRoleAssignment)
{
	chrono::_V2::system_clock::time_point start = chrono::high_resolution_clock::now();

	supplementary::ClingWrapper cw;
	cw.addKnowledgeFile("data/roleassignment/roleassignment.lp");
	cw.init();
	cw.solve();
	cw.printLastModel();
	std::chrono::_V2::system_clock::time_point end = chrono::high_resolution_clock::now();
	cout << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;

	//cw.add("base", {"r1","speak"}, "-hasCapability.");

	cw.assignExternal("-hasCapability", {"r1", "speak"}, true);
	//cw.ground("base", {});
	Gringo::SolveResult solve = cw.solve();
	if (Gringo::SolveResult::SAT == solve)
	{
		cout << "Model Found" << endl;
	}
	cw.printLastModel();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

