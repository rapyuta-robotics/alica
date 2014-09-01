/*
 * test_cling_wrap.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: Stephan Opfer
 */

#include <gtest/gtest.h>
#include "ClingWrapper.h"
#include "External.h"
#include "BaseLiteral.h"
#include "BoolLiteral.h"
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

	std::chrono::_V2::system_clock::time_point end = chrono::high_resolution_clock::now();
	cout << "Init-Time:" << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;

	for (int i = 1; i < 10; i++)
	{
		chrono::_V2::system_clock::time_point incstart = chrono::high_resolution_clock::now();
		cw.ground("board", {i});
		cw.solve();
		cw.printLastModel();
		std::chrono::_V2::system_clock::time_point incend = chrono::high_resolution_clock::now();

		cout << "Inc-Time:" << chrono::duration_cast<chrono::nanoseconds>(incend - incstart).count() << " ns" << endl;
	}
}

TEST(ClingWrap, iclingo)
{
	chrono::_V2::system_clock::time_point start = chrono::high_resolution_clock::now();

	std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
	cw->addKnowledgeFile("data/iclingo/example.lp");
	cw->init();

	std::chrono::_V2::system_clock::time_point end = chrono::high_resolution_clock::now();
	cout << "Init-Time:" << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;
	int step = 1;
	int iquery = 1;
	int imin = 1;
	int imax = 11;
	string istop = "SAT";
	std::shared_ptr<supplementary::External> external;


	while (true)
	{
		if (step > imax)
		{
			break;
		}
		cw->ground("cumulative", {step + 3});
		if (step > iquery && external)
		{
			external->release();
		}
                cw->solve();

		external = cw->getExternal("query", {step}, "volatile", {step});
		external->assign(true);

		Gringo::SolveResult ret = cw->solve();
		cw->printLastModel();
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

	auto ext = cw.getExternal("hasCapability",{"r1", "dance"});
	ext->assign(true);


	//Gringo::SolveResult solve = cw.solve();
//	if (Gringo::SolveResult::SAT == solve)
//	{
//		cout << "Model Found" << endl;
//	}
//	cw.printLastModel(true);
}

void testModel(supplementary::ClingWrapper* cw, std::map<uint32, const char*>* symbols)
{
  if (cw->getLastSolver() == nullptr)
    return;

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
        std::vector<std::shared_ptr<supplementary::External>> players;
        std::vector<std::shared_ptr<supplementary::External>> playersDeleted;
        supplementary::ClingWrapper* cw = new supplementary::ClingWrapper();
        cw->addKnowledgeFile("data/model/testModelLiterals.lp");
        cw->init();

        cw->ground("inc", {1});

        auto ext = cw->getExternal("player", {1, 1});
        ext->assign(true);

        players.push_back(ext);

        for (int i=2; i < 10; ++i)
        {
//          if ( i == 6 )
//          {
//            cw->add("showArea", {}, "#show column/1.");
//            cw->ground("showArea", {});
//          }

          cw->ground("inc", {i});

          int row = (rand() % i) + 1;
          int column = (rand() % i) + 1;

          auto rowPlayer =  cw->getExternal("player", {row, i});
          auto columnPlayer = cw->getExternal("player", {i, column});

          rowPlayer->assign(true);
          columnPlayer->assign(true);

          players.push_back(rowPlayer);
          players.push_back(columnPlayer);

          if (playersDeleted.size() > 0)
          {
            int toAdd = (rand() % playersDeleted.size());
            playersDeleted[toAdd]->assign(true);
            players.push_back(playersDeleted[toAdd]);
            playersDeleted.erase(playersDeleted.begin() + toAdd);
          }

          int r = (rand() % 10);

          if (r < 6)
          {
            int toDelete = (rand() % players.size());
            players[toDelete]->release();
            players.erase(players.begin() + toDelete);
          }
          else if (r < 8)
          {
            for (int i = 0; i < 3; ++i)
            {
              if (players.size() == 0)
                break;

              int toDelete = (rand() % players.size());
              players[toDelete]->release();
              playersDeleted.push_back(players[toDelete]);
              players.erase(players.begin() + toDelete);
            }
          }

          cw->solve();
          testModel(cw, symbols);

          //cw->printLastModel(true);
        }
}

TEST(ClingWrap, boolLiterals)
{
        supplementary::ClingWrapper* cw = new supplementary::ClingWrapper();
        cw->addKnowledgeFile("data/model/testModelLiterals.lp");
        cw->init();

        cw->ground("inc", {1});
        cw->ground("sum", {});
        auto ext = cw->getExternal("player", {1, 1});
        ext->assign(true);

        auto sum = cw->getBoolLiteral("sumPlayer", {1});
        sum->setUpdateType(supplementary::LiteralUpdateType::PUSH);

        cw->solve();
        cw->printLastModel(true);

//        EXPECT_EQ(true, sum->getValue());

        cw->ground("inc", {2});
        auto ext2 = cw->getExternal("player", {1, 2});
        ext2->assign(true);

        auto sum2 = cw->getBoolLiteral("sumPlayer", {2});

        cw->solve();
        cw->printLastModel(true);

//        EXPECT_EQ(false, sum->getValue());
//        EXPECT_EQ(true, sum2->getValue());
}

TEST(ClingWrap, composition)
{
        supplementary::ClingWrapper* cw = new supplementary::ClingWrapper();
        cw->addKnowledgeFile("data/composition/nodeComposition.lp");
        cw->init();

        // entities
        cw->ground("entity", {"entity1"});

        // systems
        auto system1 = cw->getExternal("system", {"system1", 100});
        system1->assign(true);
        auto system2 = cw->getExternal("system", {"system2", 10});
        system2->assign(true);

        // inputs
        auto input1 = cw->getExternal("inputStream", {"system1","system1", Gringo::Value("information", {"entity1","scope1"}), 0, 90, 1});
        auto input2 = cw->getExternal("inputStream", {"system1","system1",Gringo::Value("information", {"entity1","scope2"}), 5, 90, 1});
        auto input3 = cw->getExternal("inputStream", {"system2","system2",Gringo::Value("information", {"entity1","scope2"}), 2, 99, 2});

        input1->assign(true);
        input2->assign(true);
        input3->assign(true);

        // requireds
        auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1","scope3"}),-1,-1});
        required->assign(true);

        // add transfer
        auto transfer = cw->getExternal("transfer", {"system2", "system1", 1, 2});
        transfer->assign(true);

        // add node1
        cw->add("node1", {}, "#external nodeTemplate(system1,node1).");
        auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1"}, "node1", {});
        node1->assign(false);
        cw->add("node1", {}, "input(system1,node1,scope1,1,1) :- nodeTemplate(system1,node1).");
        cw->add("node1", {}, "input(system1,node1,scope2,1,1) :- nodeTemplate(system1,node1).");
        cw->add("node1", {}, "output(system1,node1,scope3).");
        cw->add("node1", {}, "nodeDelay(system1,node1,1).");
        cw->add("node1", {}, "nodeCost(system1,node1,10).");
        cw->ground("node1", {});

        // add node2
        cw->add("node2", {}, "#external nodeTemplate(system1,node2).");
        auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2"}, "node2", {});
        node2->assign(false);
        cw->add("node2", {}, "input(system1,node2,scope1,1,1) :- nodeTemplate(system1,node2).");
        cw->add("node2", {}, "input(system1,node2,scope2,2,2) :- nodeTemplate(system1,node2).");
        cw->add("node2", {}, "output(system1,node2,scope3).");
        cw->add("node1", {}, "nodeDelay(system1,node2,1).");
        cw->add("node2", {}, "nodeCost(system1,node2,5).");
        cw->ground("node2", {});

        cw->solve();
        cw->printLastModel();

        auto query1 = cw->getExternal("query", {1});
        query1->assign(true);

        cw->solve();
        cw->printLastModel();

        node2->assign(true);
        cw->solve();
        cw->printLastModel();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

