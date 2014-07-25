// {{{ GPL License 

// This file is part of gringo - a grounder for logic programs.
// Copyright (C) 2013  Benjamin Kaufmann
// Copyright (C) 2013  Roland Kaminski

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// }}}

#ifndef _CLINGO_GROUNDER_HH
#define _CLINGO_GROUNDER_HH

#include <gringo/output/output.hh>
#include <gringo/input/program.hh>
#include <gringo/input/programbuilder.hh>
#include <gringo/input/nongroundparser.hh>
#include <gringo/control.hh>
#include <gringo/logger.hh>
#include <gringo/scripts.hh>
#include <clasp/logic_program.h>
#include <clasp/clasp_facade.h>
#include <program_opts/application.h>

// #define DEBUG_OUTPUT

// {{{ declaration of ClaspLpOutput

class ClaspLpOutput : public Gringo::Output::LparseOutputter {
    public:
        ClaspLpOutput(Clasp::Asp::LogicProgram& out) : prg_(out) {
            false_ = prg_.newAtom();
            prg_.setCompute(false_, false);
#ifdef DEBUG_OUTPUT
            std::cerr << "pb.setCompute(" << false_ << ",false);\n";
#endif
        }
        unsigned falseUid() { return false_; }
        unsigned newUid()   { return prg_.newAtom(); }
        void printBasicRule(unsigned head, LitVec const &body);
        void printChoiceRule(AtomVec const &atoms, LitVec const &body);
        void printCardinalityRule(unsigned head, unsigned lower, LitVec const &body);
        void printWeightRule(unsigned head, unsigned lower, LitWeightVec const &body);
        void printMinimize(LitWeightVec const &body);
        void printDisjunctiveRule(AtomVec const &atoms, LitVec const &body);
        void finishRules()   { /* noop */ }
        void printSymbol(unsigned atomUid, Gringo::Value v);
        void printExternal(unsigned atomUid, Gringo::Output::ExternalType type);
        void finishSymbols() { /* noop */ }
        bool &disposeMinimize();

    private:
        void addBody(const LitVec& body);
        void addBody(const LitWeightVec& body);
        ClaspLpOutput(const ClaspLpOutput&);
        ClaspLpOutput& operator=(const ClaspLpOutput&);
        Clasp::Asp::LogicProgram& prg_;
        unsigned false_;
        std::stringstream str_;
        bool disposeMinimize_ = true;
};

// }}}
// {{{ declaration of GringoOptions

struct GringoOptions {
	ProgramOptions::StringSeq defines;
    Gringo::Output::LparseDebug lparseDebug;
	bool verbose         = false;
	bool text            = false;
	bool lpRewrite       = false;
	bool wNoRedef        = false;
	bool wNoCycle        = false;
	bool wNoTermUndef    = false;
	bool wNoAtomUndef    = false;
	bool wNoNonMonotone  = false;
	bool wNoFileIncluded = false;
};

// }}}
// {{{ declaration of Grounder
class Grounder {
public:
    using StringVec   = std::vector<std::string>;
    using ExternalVec = std::vector<std::pair<Gringo::Value, Gringo::Output::ExternalType>>;

	typedef ProgramOptions::StringSeq StringSeq;
	Grounder();
	void parse(const StringSeq& files, const GringoOptions& opts, Clasp::Asp::LogicProgram* out);
	void ground(Gringo::Ground::Parameters& params, Gringo::Input::ProgramVec& parts);
	void main(Gringo::Control &ctl);

    ExternalVec                                             freeze;
	std::unique_ptr<Gringo::Output::OutputBase>             out;
	std::unique_ptr<Gringo::Output::LparseOutputter>        lpOut;
    Gringo::Scripts                                         scripts;
    Gringo::Input::Program                                  prg;
    Gringo::Defines                                         defs;
	std::unique_ptr<Gringo::Input::NongroundProgramBuilder> pb;
	std::unique_ptr<Gringo::Input::NonGroundParser>         parser;
    bool                                                    verbose_ = false;
};

// }}}
// {{{ declaration of ClingoSolveFuture

Gringo::SolveResult convert(Clasp::ClaspFacade::Result res);
#if WITH_THREADS
struct ClingoSolveFuture : Gringo::SolveFuture {
    ClingoSolveFuture();
    virtual Gringo::SolveResult get();
    virtual void wait();
    virtual bool wait(double timeout);
    virtual void interrupt();
    virtual ~ClingoSolveFuture();
    void ready(Clasp::ClaspFacade::Result ret);
    void reset(Gringo::Control::FinishHandler fh);
    void reset(Clasp::ClaspFacade::AsyncResult res);
    std::unique_ptr<Clasp::ClaspFacade::AsyncResult> future;
    Gringo::Control::FinishHandler                   callback;
    Gringo::SolveResult                              ret = Gringo::SolveResult::UNKNOWN;
};
#endif

// }}}

#endif // _CLINGO_GROUNDER_HH
