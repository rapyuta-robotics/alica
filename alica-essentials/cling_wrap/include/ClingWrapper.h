/*
 * ClingWrapper.h
 *
 *  Created on: Jul 28, 2014
 *      Author: Stephan Opfer
 */

#ifndef CLINGWRAPPER_H_
#define CLINGWRAPPER_H_

using namespace std;

#include <climits>
#include <string>
#include <iostream>
#include <memory>

#include "../app/clingo/src/clingo_app.hh"

namespace Clasp {
	class Solver;
	class Model;
}

namespace supplementary
{


	class ClingWrapper : protected ClingoApp
	{
	public:
		ClingWrapper();
		virtual ~ClingWrapper();
		void init();
		void setMode(Mode mode);
		Gringo::SolveResult solve();
		void ground(std::string const &name, Gringo::FWValVec args);
		void assignExternal(shared_ptr<Gringo::Value> ext, bool val);
		shared_ptr<Gringo::Value> assignExternal(std::string const &name, Gringo::FWValVec args, bool val);
		void releaseExternal(shared_ptr<Gringo::Value> ext);
		shared_ptr<Gringo::Value> releaseExternal(std::string const &name, Gringo::FWValVec args);
		void add(std::string const &name, Gringo::FWStringVec const &params, std::string const &part);
		void addKnowledgeFile(std::string path);
		virtual	bool onModel(const Clasp::Solver& s, const Clasp::Model& m);
		void printLastModel();

	private:
		const Clasp::Model* lastModel;
		const Clasp::Solver* lastSolver;
	};

} /* namespace supplementary */

#endif /* CLINGWRAPPER_H_ */
