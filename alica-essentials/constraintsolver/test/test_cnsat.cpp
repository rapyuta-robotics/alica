#include <gtest/gtest.h>

#include <AutoDiff.h>
#include "types/Clause.h"
#include "types/Lit.h"
#include "types/Var.h"
#include "CNSat.h"
#include "FileSystem.h"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>

using namespace std;
using namespace alica::reasoner;
using namespace alica::reasoner::cnsat;

TEST(CNSatTest, CNSAT0)
{
	shared_ptr<CNSat> cns = make_shared<CNSat>();

	string path = supplementary::FileSystem::getSelfPath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/supplementary/constraintsolver/test/cnf/testen.cnf";

	cns->readFromCNFFile(path);

	/*foreach(Alica.Reasoner.CNSAT.Clause c in cns->clauses) {
	 c->print();
	 }*/
	cns->init();
	bool sf = cns->solve();
	EXPECT_TRUE(sf);

	//cout << endl;

//	if (sf)
//	{
//		cout << "Satisfiable" << endl;
//		for (shared_ptr<Var> v : *(cns->variables))
//		{
//			v->print();
//			cout << " ";
//		}
//		cout << endl;
//	}
//	else
//		cout << "Unsatisfiable" << endl;

//	cout << "------------------\n\nChecking Solution:" << endl;
	bool isSolution = true;
	for (shared_ptr<Clause> c : *(cns->clauses))
	{
		if (!c->checkSatisfied())
		{
			isSolution = false;
			/*c->print();
			 foreach(Lit l in c.Literals) {
			 Console.Write(l.Var.WatchList.Count+"\t");
			 }

			 cout << endl;
			 c.watcher[0].Lit.Var->print();
			 Console.Write("\t");
			 c.watcher[1].Lit.Var->print();
			 Console.Write("\n");*/
		}
	}
	EXPECT_TRUE(isSolution);
	cout << "Is Solution: " << isSolution << endl;
	//cns->printStatistics();
//
//	cout << endl;

}



TEST(CNSatTest, CNSATaim_50_1_6_yes1_4)
{
	shared_ptr<CNSat> cns = make_shared<CNSat>();

	string path = supplementary::FileSystem::getSelfPath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/supplementary/constraintsolver/test/cnf/aim-50-1_6-yes1-4.cnf";

	cns->readFromCNFFile(path);

	cns->init();
	bool sf = cns->solve();
	EXPECT_TRUE(sf);

	bool isSolution = true;
	for (shared_ptr<Clause> c : *(cns->clauses))
	{
		if (!c->checkSatisfied())
		{
			isSolution = false;
		}
	}
	EXPECT_TRUE(isSolution);
}


TEST(CNSatTest, CNSATpar8_1_c_cnf)
{
	shared_ptr<CNSat> cns = make_shared<CNSat>();

	string path = supplementary::FileSystem::getSelfPath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/supplementary/constraintsolver/test/cnf/par8-1-c.cnf";

	cns->readFromCNFFile(path);

	cns->init();
	bool sf = cns->solve();
	EXPECT_TRUE(sf);

	bool isSolution = true;
	for (shared_ptr<Clause> c : *(cns->clauses))
	{
		if (!c->checkSatisfied())
		{
			isSolution = false;
		}
	}
	EXPECT_TRUE(isSolution);
}



TEST(CNSatTest, CNSAT1_aim_100_1_6_no_1cnf)
{
	shared_ptr<CNSat> cns = make_shared<CNSat>();

	string path = supplementary::FileSystem::getSelfPath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/supplementary/constraintsolver/test/cnf/aim-100-1_6-no-1.cnf";

	cns->readFromCNFFile(path);

	cns->init();
	bool sf = cns->solve();
	EXPECT_FALSE(sf);
}


TEST(CNSatTest, CNSAT1dubois22)
{
	shared_ptr<CNSat> cns = make_shared<CNSat>();

	string path = supplementary::FileSystem::getSelfPath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/supplementary/constraintsolver/test/cnf/dubois22.cnf";

	cns->readFromCNFFile(path);

	cns->init();
	bool sf = cns->solve();
	EXPECT_FALSE(sf);
}


TEST(CNSatTest, CNSAThole6)
{
	shared_ptr<CNSat> cns = make_shared<CNSat>();

	string path = supplementary::FileSystem::getSelfPath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/supplementary/constraintsolver/test/cnf/hole6.cnf";

	cns->readFromCNFFile(path);

	cns->init();
	bool sf = cns->solve();
	EXPECT_FALSE(sf);
}


TEST(CNSatTest, CNSAT1_dubois20)
{
	shared_ptr<CNSat> cns = make_shared<CNSat>();

	string path = supplementary::FileSystem::getSelfPath();
	int place = path.rfind("devel");
	path = path.substr(0, place);
	path = path + "src/supplementary/constraintsolver/test/cnf/dubois20.cnf";

	cns->readFromCNFFile(path);

	cns->init();
	bool sf = cns->solve();
	EXPECT_FALSE(sf);
}
