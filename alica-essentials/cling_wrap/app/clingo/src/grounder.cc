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

#include "grounder.hh"
#include "ClingWrapper.h"

// {{{ definition of ClaspLpOutput

void ClaspLpOutput::addBody(const LitVec& body) {
    for (auto x : body) {
        prg_.addToBody((Clasp::Var)std::abs(x), x > 0);
    }
}
void ClaspLpOutput::addBody(const LitWeightVec& body) {
    for (auto x : body) {
        prg_.addToBody((Clasp::Var)std::abs(x.first), x.first > 0, x.second);
    }
}
void ClaspLpOutput::printBasicRule(unsigned head, LitVec const &body) {
    prg_.startRule().addHead(head);
    addBody(body);
    prg_.endRule();
}

void ClaspLpOutput::printChoiceRule(AtomVec const &atoms, LitVec const &body) {
    prg_.startRule(Clasp::Asp::CHOICERULE);
    for (auto x : atoms) { prg_.addHead(x); }
    addBody(body);
    prg_.endRule();
}

void ClaspLpOutput::printCardinalityRule(unsigned head, unsigned lower, LitVec const &body) {
    prg_.startRule(Clasp::Asp::CONSTRAINTRULE, lower).addHead(head);
    addBody(body);
    prg_.endRule();
}

void ClaspLpOutput::printWeightRule(unsigned head, unsigned lower, LitWeightVec const &body) {
    prg_.startRule(Clasp::Asp::WEIGHTRULE, lower).addHead(head);
    addBody(body);
    prg_.endRule();
}

void ClaspLpOutput::printMinimize(LitWeightVec const &body) {
    prg_.startRule(Clasp::Asp::OPTIMIZERULE);
    addBody(body);
    prg_.endRule();
}

void ClaspLpOutput::printDisjunctiveRule(AtomVec const &atoms, LitVec const &body) {
    prg_.startRule(Clasp::Asp::DISJUNCTIVERULE);
    for (auto x : atoms) { prg_.addHead(x); }
    addBody(body);
    prg_.endRule();
}

void ClaspLpOutput::printSymbol(unsigned atomUid, Gringo::Value v) {
    if (this->clingWrapper)
      this->clingWrapper->registerLiteral(atomUid, v);

    if (v.type() == Gringo::Value::ID || v.type() == Gringo::Value::STRING) {
            prg_.setAtomName(atomUid, (*v.string()).c_str());
    }
    else {
            str_.str("");
            v.print(str_);
            prg_.setAtomName(atomUid, str_.str().c_str());
    }
}

void ClaspLpOutput::printExternal(unsigned atomUid, Gringo::Output::ExternalType type) {
    switch (type) {
        case Gringo::Output::ExternalType::E_FALSE: { prg_.freeze(atomUid, Clasp::value_false); break; }
        case Gringo::Output::ExternalType::E_TRUE:  { prg_.freeze(atomUid, Clasp::value_true); break; }
        case Gringo::Output::ExternalType::E_FREE:  { prg_.unfreeze(atomUid); break; }
    }
}

bool &ClaspLpOutput::disposeMinimize() {
    return disposeMinimize_;
}

// }}}
// {{{ definition of Grounder

#define LOG if (verbose_) std::cerr
Grounder::Grounder() {}

void Grounder::parse(const StringSeq& files, const GringoOptions& opts, Clasp::Asp::LogicProgram* claspOut,
                     supplementary::ClingWrapper* clingWrapper) {
    using namespace Gringo;
	if (opts.wNoRedef)        { message_printer()->disable(W_DEFINE_REDEFINTION); }
	if (opts.wNoCycle)        { message_printer()->disable(W_DEFINE_CYCLIC);  }
	if (opts.wNoTermUndef)    { message_printer()->disable(W_TERM_UNDEFINED); }
	if (opts.wNoAtomUndef)    { message_printer()->disable(W_ATOM_UNDEFINED); }
	if (opts.wNoNonMonotone)  { message_printer()->disable(W_NONMONOTONE_AGGREGATE); }
	if (opts.wNoFileIncluded) { message_printer()->disable(W_FILE_INCLUDED); }
	verbose_ = opts.verbose;
	Output::OutputPredicates outPreds;
	if (opts.text) {
		out.reset(new Output::OutputBase(std::move(outPreds), std::cout, opts.lpRewrite));
	}
	else {	
		if (claspOut) { lpOut.reset(new ClaspLpOutput(*claspOut, clingWrapper)); }
		else          { lpOut.reset(new Output::PlainLparseOutputter(std::cout)); }
		out.reset(new Output::OutputBase(std::move(outPreds), *lpOut, opts.lparseDebug));
	}
	pb = make_unique<Input::NongroundProgramBuilder>(scripts, prg, *out, defs);
	parser = make_unique<Input::NonGroundParser>(*pb);
	for (auto &x : opts.defines) {
		LOG << "define: " << x << std::endl;
		parser->parseDefine(x);
	}
	for (auto x : files) {
		LOG << "file: " << x << std::endl;
		parser->pushFile(std::move(x));
	}
	if (files.empty()) {
		LOG << "reading from stdin" << std::endl;
		parser->pushFile("-");
	}
	parser->parse();
	LOG << "************** parsed program **************" << std::endl << prg;
}

void Grounder::ground(Gringo::Ground::Parameters& params, Gringo::Input::ProgramVec& parts) {
    if (!parts.empty()) {
        parser->pushBlocks(std::move(parts));
        parser->parse();
        parts.clear();
    }
	prg.rewrite(defs);
	LOG << "************* rewritten program ************" << std::endl << prg;
	prg.check();
	if (Gringo::message_printer()->hasError()) {
		throw std::runtime_error("grounding stopped because of errors");
	}
	auto gPrg = prg.toGround(out->domains);
	LOG << "*********** intermediate program ***********" << std::endl << gPrg << std::endl;
    LOG << "************* grounded program *************" << std::endl;
    gPrg.ground(params, scripts, *out, false);
    for (auto &ext : freeze) {
        Gringo::PredicateDomain::element_type *atm = out->find2(ext.first);
        if (atm && atm->second.hasUid()) {
            out->external(*atm, ext.second);
        }
    }
    freeze.clear();
    params.clear();
    out->finish();
}

void Grounder::main(Gringo::Control &ctl) {
    if (scripts.callable("main")) { 
        out->incremental();
        scripts.main(ctl);
    }
    else {
        ctl.ground("base", {});
        ctl.solve(nullptr);
    }
}

// }}}
// {{{ definition of ClingoSolveFuture

Gringo::SolveResult convert(Clasp::ClaspFacade::Result res) {
    switch (res) {
        case Clasp::ClaspFacade::Result::SAT:     { return Gringo::SolveResult::SAT; }
        case Clasp::ClaspFacade::Result::UNSAT:   { return Gringo::SolveResult::UNSAT; }
        case Clasp::ClaspFacade::Result::UNKNOWN: { return Gringo::SolveResult::UNKNOWN; }
    }
    return Gringo::SolveResult::UNKNOWN;
}

#if WITH_THREADS
ClingoSolveFuture::ClingoSolveFuture()
    : future(nullptr) { }
Gringo::SolveResult ClingoSolveFuture::get() {
    if (future) { 
        bool stop = future->interrupted() == SIGINT;
        ret       = convert(future->get());
        future    = nullptr;
        callback  = nullptr;
        if (stop) { throw std::runtime_error("solving stopped by signal"); }
    }
    return ret;
}
void ClingoSolveFuture::wait() { get(); }
bool ClingoSolveFuture::wait(double timeout) {
    if (future) {
        if (timeout == 0 ? !future->ready() : !future->waitFor(timeout)) { return false; }
        wait();
    }
    return true;
}
void ClingoSolveFuture::interrupt() {
    if (future) {
        future->cancel();
        wait();
    }
}
void ClingoSolveFuture::ready(Clasp::ClaspFacade::Result ret) {
    if (callback) { 
        callback(convert(ret), ret.interrupted());
    }
}
void ClingoSolveFuture::reset(Gringo::Control::FinishHandler fh) {
    future   = nullptr;
    callback = fh;
}
void ClingoSolveFuture::reset(Clasp::ClaspFacade::AsyncResult res) {
    future   = Gringo::make_unique<Clasp::ClaspFacade::AsyncResult>(res);
}
ClingoSolveFuture::~ClingoSolveFuture() { }
#endif

// }}}
