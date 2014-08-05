/*
 * ClingWrapper.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: Stephan Opfer
 */

#include "ClingWrapper.h"
#include "clasp/solver.h"

using namespace Clasp;
using namespace Clasp::Cli;

namespace supplementary
{

	ClingWrapper::ClingWrapper()
	{
		setMode(Mode::mode_clingo);
		this->setVerbose(0);
		claspAppOpts_.quiet[0] = 2;
		claspAppOpts_.quiet[1] = 2;
		claspAppOpts_.quiet[2] = 2;
	}

	ClingWrapper::~ClingWrapper()
	{
		// TODO Auto-generated destructor stub
	}

	void ClingWrapper::init()
	{
		this->setup();
		bool incremental = mode_ != mode_clasp;
		Clasp::ProblemType pt = getProblemType();
		ProgramBuilder* prg = &clasp_->start(claspConfig_, pt, incremental);
		if (incremental)
		{
			grOpts_.verbose = verbose() == UINT_MAX;
			Asp::LogicProgram* lp = mode_ != mode_gringo ? static_cast<Asp::LogicProgram*>(prg) : 0;
			grd = Gringo::make_unique<Grounder>();
			grd->parse(claspAppOpts_.input, grOpts_, lp);
			this->ground("base", {});
		}
	}

	void ClingWrapper::setMode(Mode mode)
	{
		this->mode_ = mode;
	}

	Gringo::SolveResult ClingWrapper::solve()
	{
		lastModel = nullptr;
		lastSolver = nullptr;
		Gringo::SolveResult result = ClingoApp::solve(nullptr);
		return result;
	}

	void ClingWrapper::ground(std::string const &name, Gringo::FWValVec args)
	{
		ClingoApp::ground(name, args);
	}

	void ClingWrapper::addKnowledgeFile(std::string path)
	{
		this->claspAppOpts_.input.push_back(path);
	}

	bool ClingWrapper::onModel(const Clasp::Solver& s, const Clasp::Model& m)
	{
		lastModel = &m;
		lastSolver = &s;
		return true;
	}

	void ClingWrapper::assignExternal(shared_ptr<Gringo::Value> ext, bool val)
	{
		ClingoApp::assignExternal(*ext, val);
	}

	shared_ptr<Gringo::Value>ClingWrapper::ClingWrapper::assignExternal(const std::string& name, Gringo::FWValVec args, bool val)
	{
		shared_ptr<Gringo::Value> gv = (args.size() == 0 ? make_shared<Gringo::Value>(name) : make_shared<Gringo::Value>(name, args));
		assignExternal(gv, val);
		return gv;
	}

	void ClingWrapper::releaseExternal(shared_ptr<Gringo::Value> ext)
	{
		ClingoApp::releaseExternal(*ext);
	}

	shared_ptr<Gringo::Value> ClingWrapper::releaseExternal(std::string const &name, Gringo::FWValVec args)
	{
		shared_ptr<Gringo::Value> gv = (args.size() == 0 ? make_shared<Gringo::Value>(name) : make_shared<Gringo::Value>(name, args));
		releaseExternal (gv);
		return gv;
	}

	void ClingWrapper::add(const std::string& name, const Gringo::FWStringVec& params, const std::string& part)
	{
		ClingoApp::add(name, params, part);
	}

	void ClingWrapper::printLastModel(bool verbose)
	{
	        std::cout << "Last Model ";

		if (!lastModel)
		{
			std::cout << "No model found" << std::endl;
			return;
		}

		for (auto value : this->lastSolver->symbolTable())
		{
			if (this->lastModel->isTrue(value.second.lit))
				std::cout << value.second.name.c_str() << " ";
		}

		std::cout << std::endl;

		if (verbose)
		{
		        std::cout << "Symbol table: ";
		        for (auto value : this->lastSolver->symbolTable())
                        {
                                std::cout << "(" << value.second.name.c_str() << ", " <<
                                    (this->lastModel->isTrue(value.second.lit) ? "true" : "false") << ") ";
                        }

	                std::cout << std::endl;
		}
	}

        const Clasp::Model* ClingWrapper::getLastModel()
        {
          return this->lastModel;
        }

        const Clasp::Solver* ClingWrapper::getLastSolver()
        {
          return this->lastSolver;
        }

} /* namespace supplementary */

