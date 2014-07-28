/*
 * ClingWrapper.h
 *
 *  Created on: Jul 28, 2014
 *      Author: Stephan Opfer
 */

#ifndef CLINGWRAPPER_H_
#define CLINGWRAPPER_H_

#include <climits>
#include <string>
#include <iostream>

#include "../app/clingo/src/clingo_app.hh"

namespace Clasp {
	class Solver;
	class Model;
}

namespace supplementary
{


	class ClingWrapper : public ClingoApp
	{
	public:
		ClingWrapper();
		virtual ~ClingWrapper();
		void init();
		void setMode(Mode mode);
		void solve();
		void addKnowledgeFile(std::string path);
		virtual	bool onModel(const Clasp::Solver& s, const Clasp::Model& m);
		void printLastModel();

	private:
		const Clasp::Model* lastModel;
		const Clasp::Solver* lastSolver;
	};

} /* namespace supplementary */

#endif /* CLINGWRAPPER_H_ */
