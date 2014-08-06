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
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "../app/clingo/src/clingo_app.hh"

namespace Clasp {
	class Solver;
	class Model;
}

namespace supplementary
{
  class External;
}

namespace supplementary
{


	class ClingWrapper : protected ClingoApp, public enable_shared_from_this<ClingWrapper>
	{
	  friend class External;

	public:
		ClingWrapper();
		virtual ~ClingWrapper();
		void init();
		void setMode(Mode mode);
		Gringo::SolveResult solve();
		void ground(std::string const &name, Gringo::FWValVec args);
		void add(std::string const &name, Gringo::FWStringVec const &params, std::string const &part);
		void addKnowledgeFile(std::string path);
		virtual	bool onModel(const Clasp::Solver& s, const Clasp::Model& m);
		void printLastModel(bool verbose = false);

		std::shared_ptr<External> const getExternal(std::string const &name, Gringo::FWValVec args,
		                                            std::string const &ground);
                std::shared_ptr<External> const getExternal(std::string const &name, Gringo::FWValVec args,
                                                            std::string const &ground, Gringo::FWValVec groundArgs);

		const Clasp::Model* getLastModel();
		const Clasp::Solver* getLastSolver();

	private:
                void assignExternal(shared_ptr<Gringo::Value> ext, bool val);
                shared_ptr<Gringo::Value> assignExternal(std::string const &name, Gringo::FWValVec args, bool val);
                void releaseExternal(shared_ptr<Gringo::Value> ext);
                shared_ptr<Gringo::Value> releaseExternal(std::string const &name, Gringo::FWValVec args);

	private:
		std::map<size_t, std::shared_ptr<External>> externals;
                std::mutex mtx_;
		const Clasp::Model* lastModel;
		const Clasp::Solver* lastSolver;
	};

} /* namespace supplementary */

#endif /* CLINGWRAPPER_H_ */
