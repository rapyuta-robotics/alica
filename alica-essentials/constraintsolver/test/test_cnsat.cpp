#include <gtest/gtest.h>

#include <AutoDiff.h>
#include "types/Clause.h"
#include "types/Lit.h"
#include "types/Var.h"
#include "CNSat.h"

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

	ifstream input("/home/psp/cnws/src/supplementary/constraintsolver/test/cnf/testen.cnf");
	for (string line; getline(input, line);)
	{
		if (line[0] == 'c')
			continue;
		if (line[0] == 'p')
			continue;

		shared_ptr<Clause> c = make_shared<Clause>();

		stringstream ss(line);
		string s;
		while (!ss.eof())
		{
			ss >> s;
			istringstream is(s);
			int val;
			is >> val;

			if (val == 0)
				continue;
			int valn = (val < 0) ? -val : val;
			while (cns->variables->size() < valn)
				cns->newVar();

			shared_ptr<Lit> l = make_shared<Lit>(cns->variables->at(valn - 1), Assignment::TRUE);
			if (val < 0)
			{
				l->sign = Assignment::FALSE;
			}
			cns->variables->at(valn - 1)->preferedSign = (val > 0);
			c->add(l);
		}
		cns->addBasicClause(c);
	}

	/*foreach(Alica.Reasoner.CNSAT.Clause c in cns->clauses) {
	 c->print();
	 }*/
	cns->init();
	bool sf = cns->solve();

	cout << endl;

	if (sf)
	{
		cout << "Satisfiable" << endl;
		for (shared_ptr<Var> v : *(cns->variables))
		{
			v->print();
			cout << " ";
		}
		cout << endl;
	}
	else
		cout << "Unsatisfiable" << endl;

	cout << "------------------\n\nChecking Solution:" << endl;
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
	cout << "Is Solution: " << isSolution << endl;
	cns->printStatistics();

	cout << endl;

	input.close();
}
