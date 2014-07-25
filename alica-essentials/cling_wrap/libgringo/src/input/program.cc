// {{{ GPL License 

// This file is part of gringo - a grounder for logic programs.
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

#include "gringo/input/program.hh"
#include "gringo/input/literals.hh"
#include "gringo/input/statements.hh"
#include "gringo/input/aggregates.hh"
#include "gringo/ground/literal.hh"
#include "gringo/ground/statements.hh"
#include "gringo/term.hh"
#include "gringo/logger.hh"
#include "gringo/graph.hh"
#include "gringo/safetycheck.hh"

namespace Gringo { namespace Input {

// {{{ definition of Block

Block::Block(Location const &loc, FWString name, IdVec &&params)
    : loc(loc)
    , name(name)
    , params(std::move(params))
    , edb(std::make_shared<Ground::SEdb::element_type>(nullptr, ValVec())) { 
    UTermVec args;
    for (auto &param : this->params) { args.emplace_back(make_locatable<ValTerm>(param.first, param.second)); }
    if (args.empty()) { std::get<0>(*edb) = make_locatable<ValTerm>(loc, name); }
    else              { std::get<0>(*edb) = make_locatable<FunctionTerm>(loc, name, std::move(args)); }
}

Term const &Block::sig() const {
    return *std::get<0>(*edb);
}
Block::operator Term const &() const { return sig(); }
    
// }}}
// {{{ definition of Program

Program::Program() {
    begin(Location("<internal>", 1, 1, "<internal>", 1, 1), "base", IdVec({}));
}

Program::Program(Program &&) = default;

void Program::begin(Location const &loc, FWString name, IdVec &&params) {
    current_ = &*blocks_.emplace_back(loc, "#inc_" + *name, std::move(params)).first;
}

void Program::add(UStm &&stm) {
    current_->addedEdb.emplace_back(stm->isEDB());
    if (current_->addedEdb.back().type() == Value::SPECIAL) {
        current_->addedStms.emplace_back(std::move(stm));
        current_->addedEdb.pop_back();
    }
}

void Program::rewrite(Defines &defs) {
    defs.init();
    for (auto &block : blocks_) {
        Defines incDefs;

        UTermVec args;
        unsigned incNum{0};
        for (auto &param : block.params) {
            args.emplace_back(Term::uniqueVar(param.first, incNum, 0, "#Inc"));
            incDefs.add(param.first, param.second, get_clone(args.back()), false);
        }
        sigs_.emplace_back(block.name, args.size());
        UTerm blockTerm(args.empty()
            ? (UTerm)make_locatable<ValTerm>(block.loc, block.name)
            : make_locatable<FunctionTerm>(block.loc, block.name, get_clone(args)));
        incDefs.init();

        for (auto &fact : block.addedEdb) { sigs_.emplace_back(fact.sig()); }
        auto replace = [&](Defines &defs, Value fact) -> Value {
            if (defs.empty() || fact.type() == Value::SPECIAL) { return fact; }
            UTerm rt;
            Value rv;
            defs.apply(fact, rv, rt, false);
            if (rt) {
                Location loc{rt->loc()};
                block.addedStms.emplace_back(make_locatable<Rule>(loc, make_unique<SimpleHeadLiteral>(make_locatable<PredicateLiteral>(loc, NAF::POS, std::move(rt))), UBodyAggrVec{}, false));
                return Value();
            }
            else if (rv.type() != Value::SPECIAL) { return rv; }
            else { return fact; }
        };
        if (!defs.empty() || !incDefs.empty()) {
            for (auto &fact : block.addedEdb) {
                Value rv = replace(incDefs, replace(defs, fact));
                if (rv.type() != Value::SPECIAL) { std::get<1>(*block.edb).emplace_back(rv); }
            }
            block.addedEdb.clear();
        }
        else if (std::get<1>(*block.edb).empty()) { std::swap(std::get<1>(*block.edb), block.addedEdb); }
        else { std::copy(block.addedEdb.begin(), block.addedEdb.end(), std::back_inserter(std::get<1>(*block.edb))); }
        Statement::SplitVec splits;
        auto rewrite2 = [&](UStm &x) -> void { 
            x->rewrite2(splits);
            std::get<1>(*block.edb).emplace_back(x->isEDB());
            if (std::get<1>(*block.edb).back().type() == Value::SPECIAL) {
                block.stms.emplace_back(std::move(x));
                std::get<1>(*block.edb).pop_back();
            }
            else { sigs_.emplace_back(std::get<1>(*block.edb).back().sig()); }
        };
        auto rewrite1 = [&](UStm &x) -> void {
            x->rewrite1(project_);
            if (x->hasPool(false)) { for (auto &y : x->unpool(false)) { rewrite2(y); } }
            else                   { rewrite2(x); } 
        };
        for (auto &x : block.addedStms) {
            x->replace(defs);
            x->replace(incDefs);
            x->add(make_locatable<PredicateLiteral>(block.loc, NAF::POS, get_clone(blockTerm)));
            if (x->hasPool(true)) { for (auto &y : x->unpool(true)) { rewrite1(y); } }
            else                  { rewrite1(x); }
        }
        for (auto &x : splits) {
            Location loc(x.first->loc());
            block.stms.emplace_back(make_locatable<Rule>(
                loc,
                make_unique<SimpleHeadLiteral>(std::move(x.first)),
                std::move(x.second), false));
        }
        block.addedStms.clear();
    }
    for (auto &x : project_) {
        if (!x.done) {
            Location loc(x.project->loc());
            UBodyAggrVec body;
            body.emplace_back(make_unique<SimpleBodyLiteral>(make_locatable<ProjectionLiteral>(loc, get_clone(x.project))));
            stms_.emplace_back(make_locatable<Rule>(
                loc,
                make_unique<SimpleHeadLiteral>(make_locatable<PredicateLiteral>(loc, NAF::POS, get_clone(x.projected))),
                std::move(body), false));
            x.done = true;
        }
    }
}

bool Program::check() {
    bool ret = true;
    for (auto &block : blocks_) {
        for (auto &stm : block.stms) { ret = stm->check() && ret; }
    }
    return ret;
}

void Program::addClassicalNegation(FWSignature x) {
    neg_.emplace_back(x);
}

void Program::print(std::ostream &out) const {
    for (auto &block : blocks_) {
        for (auto &x : block.addedEdb)          { out << x << "." << "\n"; }
        for (auto &x : std::get<1>(*block.edb)) { out << x << "." << "\n"; }
        for (auto &x : block.addedStms)         { out << *x << "\n"; }
        for (auto &x : block.stms)              { out << *x << "\n"; }
    }
    for (auto &x : stms_) { out << *x << "\n"; }
}

Ground::Program Program::toGround(PredDomMap &domains) {
    Ground::UStmVec stms;
    stms.emplace_back(make_locatable<Ground::ExternalRule>(Location("#external", 1, 1, "#external", 1, 1)));
    ToGroundArg arg(auxNames_, domains);
    Ground::SEdbVec edb;
    for (auto &block : blocks_) {
        edb.emplace_back(block.edb);
        for (auto &x : block.stms) { x->toGround(arg, stms); }
    }
    for (auto &x : stms_) { x->toGround(arg, stms); }
    Ground::Statement::Dep dep;
    for (auto &x : stms) {
        bool normal(x->isNormal());
        auto &node(dep.add(std::move(x), normal));
        node.stm->analyze(node, dep);
    }
    Ground::Program::ClassicalNegationVec negate;
    for (auto &x : neg_) { negate.emplace_back(Gringo::add(domains, x), Gringo::add(domains, Signature("-" + *(*x).name(), (*x).length()))); }
    Ground::Program prg(std::move(edb), dep.analyze(), std::move(negate));
    for (auto &sig : sigs_) {
        if (domains.find(*sig) == domains.end()) { domains.emplace_back(std::piecewise_construct, std::forward_as_tuple(*sig), std::forward_as_tuple()); }
    }
    for (auto &x : dep.depend.occs) {
        for (auto &y : x.second.first->depend) { (*std::get<0>(y)).checkDefined(locs_, sigs_); }
    }
    return prg;
}

Program::~Program() { }

std::ostream &operator<<(std::ostream &out, Program const &p) {
    p.print(out);
    return out;
}

// }}}

} } // namespace Input Gringo
