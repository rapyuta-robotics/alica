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

	void ClingWrapper::solve()
	{
		lastModel = nullptr;
		lastSolver = nullptr;
		Gringo::SolveResult clingoApp = ClingoApp::solve(nullptr);
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

	void ClingWrapper::printLastModel()
	{
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
	}

} /* namespace supplementary */

