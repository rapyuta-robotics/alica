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

#include "gringo/ground/program.hh"
#include "gringo/output/output.hh"

namespace Gringo { namespace Ground {

// {{{ definition of Parameters

Parameters::Parameters() = default;
Parameters::~Parameters() { }
void Parameters::add(FWString name, FWValVec args) {
    params[FWSignature("#inc_" + *name, args.size())].insert(args);
}
bool Parameters::find(FWSignature sig) const {
    auto it = params.find(sig);
    return it != params.end() && !it->second.empty();
}
ParamSet::const_iterator Parameters::begin() const { return params.begin(); }
ParamSet::const_iterator Parameters::end() const   { return params.end(); }
bool Parameters::empty() const                     { return params.empty(); }
void Parameters::clear() { params.clear(); }

// }}}
// {{{ definition of Program

Program::Program(SEdbVec &&edb, Statement::Dep::ComponentVec &&stms, ClassicalNegationVec &&negate)
    : edb(std::move(edb))
    , stms(std::move(stms))
    , negate(std::move(negate)) { }

std::ostream &operator<<(std::ostream &out, Program const &p) {
    bool comma = false;
    for (auto &component : p.stms) {
        if (comma) { out << "\n"; }
        else       { comma = true; }
        out << "%" << (component.second ? " positive" : "") <<  " component";
        for (auto &stm : component.first) { out << "\n" << *stm; }
    }
    return out;
}

void Program::linearize(Scripts &scripts) {
    for (auto &x : stms) {
        for (auto &y : x.first) { y->startLinearize(true); }
        for (auto &y : x.first) { y->linearize(scripts, x.second); }
        for (auto &y : x.first) { y->startLinearize(false); }
    }
    linearized = true;
}

void Program::ground(Scripts &scripts, Output::OutputBase &out) {
    Parameters params;
    params.add("base", FWValVec({}));
    ground(params, scripts, out);
}

void Program::ground(Parameters const &params, Scripts &scripts, Output::OutputBase &out, bool finalize) {
    for (auto &x : out.domains) {
        std::string const &name = *(*x.first).name();
        if (name.compare(0, 6, "#split") == 0 || name.compare(0, 5, "#inc_") == 0) {
            // clear projection, split, and incremental domains
            x.second.clear();
        }
        else {
            // mark the incremental step in all other domains
            x.second.exports.incNext();
        }
    }
    out.checkOutPreds();
    for (auto &x : edb) {
        if (params.find(std::get<0>(*x)->getSig())) {
            for (auto &z : std::get<1>(*x)) { out.output(z); }
        }
    }
    for (auto &p : params) {
        auto base = out.domains.find(p.first);
        if (base != out.domains.end()) {
            for (auto &args : p.second) { 
                if (args.size() == 0) { base->second.insert(Value((*p.first).name()), true); }
                else { base->second.insert(Value((*p.first).name(), args), true); }
            }
        }
    }
    for (auto &x : out.domains) { 
        x.second.mark(); 
        x.second.unmark();
        x.second.expire();
    }
    Queue q;
    for (auto &x : stms) {
        if (!linearized) {
            for (auto &y : x.first) { y->startLinearize(true); }
            for (auto &y : x.first) { y->linearize(scripts, x.second); }
            for (auto &y : x.first) { y->startLinearize(false); }
        }
        // std::cerr << "============= component ===========" << std::endl;
        for (auto &y : x.first) { 
            // std::cerr << "  enqueue: " << *y << std::endl;
            y->enqueue(q);
        }
        q.process(out);
    }
    Output::PredicateLiteral pPos, pNeg;
    for (auto &x : negate) {
        for (auto it(std::get<1>(x).exports.begin() + std::get<1>(x).exports.incOffset), ie(std::get<1>(x).exports.end()); it != ie; ++it) {
            PredicateDomain::element_type &neg(*it);
            Value v = neg.first.type() == Value::ID
                ? Value((*neg.first.string()).substr(1))
                : Value((*neg.first.name()).substr(1), neg.first.args());
            auto pos(std::get<0>(x).domain.find(v));
            if (pos != std::get<0>(x).domain.end() && pos->second.defined()) {
                pPos.repr = &*pos;
                pNeg.repr = &neg;
                pPos.naf  = pNeg.naf = NAF::POS;
                out.tempRule.head = nullptr;
                out.tempRule.body.clear();
                out.tempRule.body.emplace_back(pPos);
                out.tempRule.body.emplace_back(pNeg);
                out.output(out.tempRule);
            }
        }
    }
    out.flush();
    if (finalize) { out.finish(); }
    linearized = true;
}

// }}}

} } // namespace Ground Gringo
