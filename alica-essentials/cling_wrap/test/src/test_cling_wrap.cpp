/*
 * test_cling_wrap.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: Stephan Opfer
 */

#include <gtest/gtest.h>
#include "ClingWrapper.h"
#include "clasp/solver.h"
#include <chrono>
#include <map>
#include <vector>

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
	cw.printLastModel(true);
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
	cw.printLastModel(true);
}

void testModel(supplementary::ClingWrapper* cw, std::map<uint32, const char*>* symbols)
{
  for (auto sym : cw->getLastSolver()->symbolTable())
  {
    try {
      auto inMap = symbols->at(sym.first);
      if (inMap == NULL)
      {
        symbols->insert( std::pair<uint32, const char*>(sym.first,sym.second.name.c_str()));
      }
      else
      {
        EXPECT_EQ(inMap, sym.second.name.c_str());
      }
    } catch (std::out_of_range &e) {
      symbols->insert( std::pair<uint32, const char*>(sym.first,sym.second.name.c_str()));
    }
  }
}

TEST(ClingWrap, modelStable)
{
        std::map<uint32, const char*>* symbols = new std::map<uint32, const char*>();
        std::vector<std::shared_ptr<Gringo::Value>> players;
        supplementary::ClingWrapper* cw = new supplementary::ClingWrapper();
        cw->addKnowledgeFile("data/model/testModelLiterals.lp");
        cw->init();

        cw->ground("inc", {1});
        cw->ground("player", {1, 1});

        players.push_back(cw->assignExternal("player", {1, 1}, true));

        for (int i=2; i < 10000; ++i)
        {
          cw->ground("inc", {i});

          int row = (rand() % i) + 1;
          int column = (rand() % i) + 1;

          cw->ground("player", {row, i});
          cw->ground("player", {i, column});

          auto rowPlayer = cw->assignExternal("player", {row, i}, true);
          auto columnPlayer = cw->assignExternal("player", {i, column}, true);

          players.push_back(rowPlayer);
          players.push_back(columnPlayer);

          int r = (rand() % 10);

          if (r < 6)
          {
            int toDelete = (rand() % players.size());
            cw->releaseExternal(players[toDelete]);
            players.erase(players.begin() + toDelete);
          }
          else if (r < 8)
          {
            for (int i = 0; i < 3; ++i)
            {
              if (players.size() == 0)
                break;

              int toDelete = (rand() % players.size());
              cw->releaseExternal(players[toDelete]);
              players.erase(players.begin() + toDelete);
            }
          }

          cw->solve();
          testModel(cw, symbols);
        }
}


//TEST(ClingWrap, snapshot)
//{
//  chrono::_V2::system_clock::time_point start = chrono::high_resolution_clock::now();
//
//          supplementary::ClingWrapper cw;
//          cw.addKnowledgeFile("data/queens/incqueens.lp");
//          cw.init();
//
//          for (int i=1; i < 6; ++i)
//          {
//            cw.ground("board", {i});
//          }
//          cout << "First model, board size 5" << endl;
//          cw.solve();
//          cw.printLastModel();
//
//          auto prg = cw.getProgram();
//
//          cout << "Second model, board size 6" << endl;
//          cw.ground("board", {6});
//          cw.solve();
//          cw.printLastModel();
//
//          cout << "Third model, board size 5?" << endl;
//          cw.setProgram(prg);
//          cw.solve();
//          cw.printLastModel();
//
//          std::chrono::_V2::system_clock::time_point end = chrono::high_resolution_clock::now();
//          cout << "Init-Time:" << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;
//
//
//}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

