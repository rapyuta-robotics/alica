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

#include "gringo/ground/statements.hh"
#include "gringo/safetycheck.hh"
#include "gringo/output/literals.hh"
#include "gringo/output/output.hh"
#include "gringo/ground/binders.hh"
#include "gringo/logger.hh"
#include <limits>

namespace Gringo { namespace Ground {

// {{{ definition of HeadDefinition

HeadDefinition::HeadDefinition(UTerm &&repr) : repr(std::move(repr)) { }
UGTerm HeadDefinition::getRepr() const {
    return repr->gterm();
}
void HeadDefinition::defines(IndexUpdater &update, Instantiator *inst) {
    auto ret(offsets.emplace(&update, enqueueVec.size()));
    if (ret.second)     { enqueueVec.emplace_back(&update, RInstVec{}); }
    if (active && inst) { enqueueVec[ret.first->second].second.emplace_back(*inst); }
}
void HeadDefinition::enqueue(Queue &queue) {
    for (auto &x : enqueueVec) {
        if (x.first->update()) {
            for (Instantiator &y : x.second) { y.enqueue(queue); }
        }
    }
}
void HeadDefinition::collectImportant(Term::VarSet &vars) {
    VarTermBoundVec x;
    repr->collect(x, false);
    for (auto &occ : x) { vars.emplace(occ.first->name); }
}
HeadDefinition::~HeadDefinition() { }

// }}}

// {{{ definition of *Domain::*Domain

BodyAggregateDomain::BodyAggregateDomain(UTerm &&repr, BoundVec &&bounds, AggregateFunction fun) 
    : repr(std::move(repr))
    , loc(&this->repr->loc())
    , bounds(std::move(bounds))
    , fun(fun) {
    switch (fun) {
        case AggregateFunction::COUNT:
        case AggregateFunction::SUMP:
        case AggregateFunction::MAX: {
            for (auto &x : this->bounds) {
                if (x.rel != Relation::GT && x.rel != Relation::GEQ) { positive = false; break; }
            }
            break;
        }
        case AggregateFunction::MIN: {
            for (auto &x : this->bounds) {
                if (x.rel != Relation::LT && x.rel != Relation::LEQ) { positive = false; break; }
            }
            break;
        }
        default: { positive = false; break; }
    }
}
AssignmentAggregateDomain::AssignmentAggregateDomain(UTerm &&repr, UTerm &&specialRepr, AggregateFunction fun) 
    : repr(std::move(repr))
    , specialRepr(std::move(specialRepr))
    , loc(&this->repr->loc())
    , fun(fun) { }

ConjunctionDomain::ConjunctionDomain(UTerm &&domRep, PredicateDomain *predDom, UTerm &&predRep, PredicateDomain *headDom, UTerm &&headRep, ULitVec &&lits)
    : rep(std::move(domRep))
    , head(predDom ? make_unique<ConjunctionHead>(*predDom, std::move(predRep), *headDom, std::move(headRep)) : nullptr)
    , lits(std::move(lits)) { }

HeadAggregateDomain::HeadAggregateDomain(UTerm &&repr, AggregateFunction fun, BoundVec &&bounds, FWString dummy) 
    : repr(std::move(repr))
    , fun(fun)
    , bounds(std::move(bounds)) 
    , dummy(dummy) { }

DisjunctionDomain::DisjunctionDomain(UTerm &&repr) 
    : repr(std::move(repr)) { }

DisjointDomain::DisjointDomain(UTerm &&repr) 
    : repr(std::move(repr)) { }

// }}}
// {{{ definition of *Domain::mark

void BodyAggregateDomain::mark() { ++marks; }
void AssignmentAggregateDomain::mark() { ++marks; }
void ConjunctionDomain::mark() { ++marks; }
void HeadAggregateDomain::mark() { marks.push_back(exports.size()); }
void HeadAggregateDomain::accumulateMark() { ++blocked; }
void DisjunctionDomain::mark() { marks.push_back(exports.size()); }
void DisjointDomain::mark() { ++marks; }

// }}}
// {{{ definition of *Domain::unmark

namespace Debug {

auto f = [](std::ostream &out, BodyAggregateState::Bounds::Interval const &x) {
    out 
        << (x.left.inclusive ? "[" : "(") 
        << x.left.bound
        << ","
        << x.right.bound
        << (x.right.inclusive ? "]" : ")");
};

void print (BodyAggregateState::Bounds const &x) {
    std::cerr << "{";
    print_comma(std::cerr, x.vec, ",", f);
    std::cerr << "}";
}

}

void BodyAggregateDomain::unmark() { 
    if (!--marks && !blocked) {
        unsigned generation = exports.nextGeneration;
        auto jt(exports.begin() + generation);
        for (auto it = jt, ie = exports.end(); it != ie; ++it) {
            auto dom(it->get().second.range(fun));
//            std::cerr << "  ";
//            Debug::print(it->get().second.bounds);
//            std::cerr << " contains ";
//            Debug::f(std::cerr, dom);
//            std::cerr << "?" << std::endl;
            if (it->get().second.bounds.intersects(dom)) {
                it->get().second.generation(generation++);
                it->get().second.state     = BodyAggregateState::DEFINED;
                it->get().second._positive = positive;
                it->get().second._fact     = it->get().second.bounds.contains(dom);
//                bool fact{it->get().second.bounds.contains(dom)};
//                std::cerr << "  " << it->get().first << " matches";
//                if (fact) { std::cerr << " and is fact"; }
//                std::cerr << std::endl;
                if (it != jt) { *jt = *it; }
                ++jt;
            }
            else { it->get().second.state = BodyAggregateState::OPEN; }
        }
        exports.exports.erase(jt, exports.end()); // TODO: make a function
        exports.offset = exports.size();
//        std::cerr << "removing: " << std::distance(jt, exports.end()) << " of " << exports.size() << std::endl;
    }
}

template <class Vec>
void AssignmentAggregateDomain::insert(Vec const &values, DataMap::value_type &x) {
    Value as(x.first);
    if (as.type() == Value::FUNC) { valsCache.assign(as.args().begin(), as.args().end()); }
    valsCache.emplace_back();
    for (auto &y : values) {
        valsCache.back() = y;
        auto ret(domain.emplace(std::piecewise_construct, std::forward_as_tuple(Value(as.name(), valsCache)), std::forward_as_tuple(&x.second, exports.size())));
        if (ret.second) { exports.append(*ret.first); }
//        std::cerr << "insert: " << ret.first->first << std::endl;
    }
    x.second.fact = values.size() == 1;

}
void AssignmentAggregateDomain::unmark() {
    if (!--marks && !blocked) {
//        std::cerr << "import assignments [" << offset << "," << dataVec.size() << "] of " << dataMap.size() << std::endl;
        for (auto it(dataVec.begin() + offset), ie(dataVec.end()); it != ie; ++it) {
//            std::cerr << "  assignment with " << it->get().second.elems.size() << " elements" << std::endl;
            switch (fun) {
                case AggregateFunction::MIN: {
                    ValVec values;
                    values.clear();
                    Value min(false);
                    for (auto &x : it->get().second.elems) {
                        if (x.second.empty()) {
                            Value weight(*x.first.begin());
                            if (weight < min) { min = weight; }
                        }
                    }
                    values.emplace_back(min);
                    for (auto &x : it->get().second.elems) {
                        Value weight(*x.first.begin());
                        if (weight < min) { values.emplace_back(weight); }
                    }
                    std::sort(values.begin(), values.end());
                    values.erase(std::unique(values.begin(), values.end()), values.end());
                    insert(values, *it);
                    break;
                }
                case AggregateFunction::MAX: {
                    ValVec values;
                    values.clear();
                    Value max(true);
                    for (auto &x : it->get().second.elems) {
                        if (x.second.empty()) {
                            Value weight(*x.first.begin());
                            if (max < weight) { max = weight; }
                        }
                    }
                    values.emplace_back(max);
                    for (auto &x : it->get().second.elems) {
                        Value weight(*x.first.begin());
                        if (max < weight) { values.emplace_back(weight); }
                    }
                    std::sort(values.begin(), values.end());
                    values.erase(std::unique(values.begin(), values.end()), values.end());
                    insert(values, *it);
                    break;
                }
                default: {
                    // Note: Count case could be optimized
                    std::vector<int> values;
                    values.emplace_back(0);
                    for (auto &x : it->get().second.elems) {
                        int weight = fun == AggregateFunction::COUNT ? 1 : x.first.begin()->num();
                        if (x.second.empty()) {
                            for (auto &x : values) { x+= weight; }
                        }
                        else {
                            values.reserve(values.size() * 2);
                            for (auto &x : values) { values.emplace_back(x + weight); }
                            std::sort(values.begin(), values.end());
                            values.erase(std::unique(values.begin(), values.end()), values.end());
                        }
                    }
                    insert(values, *it);
                    break;
                }
            }
        }
//        std::cerr << "done mext import is [" << exports.offset << "," << exports.size() << "]" << std::endl;
        exports.offset = exports.size();
    }
}

void ConjunctionDomain::unmark() { 
    if (!--marks && !blocked) {
        unsigned generation = exports.nextGeneration;
        auto jt(exports.begin() + generation);
        for (auto it = jt, ie = exports.end(); it != ie; ++it) {
            if (it->get().second.blocked.empty()) {
                it->get().second.generation(generation++);
                if (it != jt) { *jt = *it; }
                ++jt;
            }
            else { it->get().second._generation = 0; }
        }
        exports.exports.erase(jt, exports.end()); // TODO: make a function
        exports.offset = exports.size();
    }
}
void HeadAggregateDomain::accumulateUnmark(Queue &queue) { 
    if (!--blocked) {
        using ImportVec = std::vector<std::vector<std::reference_wrapper<PredicateDomain::element_type>>>;
        ImportVec import(heads.size());
        for (HeadAggregateState &x : todo) {
            if (x.bounds.intersects(x.range(fun))) {
                for (auto &y : x.elems) {
                    for (auto it(y.second.conds.begin() + y.second.imported), ie(y.second.conds.end()); it != ie; ++it) {
                        if (it->head) { import[it->headNum-1].emplace_back(*it->head); }
                    }
                    y.second.imported = y.second.conds.size();
                }
            }
            x.todo = false;
        }
        todo.clear();
        auto headIt(heads.begin());
        for (auto &x : import) { 
            auto &head = *headIt++;
            if (!x.empty()) {
                for (PredicateDomain::element_type &y : x) { head.first.insert(y); }
                head.first.mark();
                head.first.unmark();
            }
        }
    }
    for (auto &head : heads) { 
        head.second.enqueue(queue);
        queue.enqueue(head.first);
    }
}
void HeadAggregateDomain::unmark() { 
    assert(!marks.empty());
    exports.offset = marks.front();
    marks.pop_front();
}

void DisjunctionDomain::unmark() { 
    assert(!marks.empty());
    exports.offset = marks.front();
    marks.pop_front();
}

void DisjointDomain::unmark() { 
    if (!--marks && !blocked) {
        unsigned generation = exports.nextGeneration;
        auto jt(exports.begin() + generation);
        for (auto it = jt, ie = exports.end(); it != ie; ++it) {
            it->get().second.generation(generation++);
            if (it != jt) { *jt = *it; }
            ++jt;
        }
        exports.exports.erase(jt, exports.end()); 
        exports.offset = exports.size();
    }
}

// }}}
// {{{ definition of *Domain::insert

void init(BoundVec const &bounds, IntervalSet<Value> &set) {
    set.add({{Value(true),true}, {Value(false),true}});
    for (auto &x : bounds) {
        Value v(x.bound->eval());
        switch (x.rel) {
            case Relation::GEQ: { set.remove({{Value(true),true},{v,false}          }); break; }
            case Relation::GT:  { set.remove({{Value(true),true},{v,true}           }); break; }
            case Relation::LEQ: { set.remove({{v,false},         {Value(false),true}}); break; }
            case Relation::LT:  { set.remove({{v,true},          {Value(false),true}}); break; }
            case Relation::NEQ: { set.remove({{v,true},          {v,true}           }); break; }
            case Relation::ASSIGN: 
            case Relation::EQ: {
                set.remove({{v,false},         {Value(false),true}});
                set.remove({{Value(true),true},{v,false}          });
                break;
            }
        }
    }
}

BodyAggregateDomain::element_type &BodyAggregateDomain::reserve(Value repr) {
    auto state = domain.find(repr);
    if (state == domain.end()) { 
        state = domain.emplace(std::piecewise_construct, std::forward_as_tuple(repr), std::forward_as_tuple()).first;
        state->second.init(fun);
        Gringo::Ground::init(bounds, state->second.bounds);
        exports.append(*state);
        state->second.state = BodyAggregateState::UNKNOWN;
    }
    return *state;
}

void BodyAggregateDomain::insert(Value const &repr, ValVec const &tuple, Output::LitVec const &cond, Location const &loc) {
    auto state = domain.find(repr);
    if (state == domain.end()) { 
        state = domain.emplace(std::piecewise_construct, std::forward_as_tuple(repr), std::forward_as_tuple()).first;
        state->second.init(fun);
        Gringo::Ground::init(bounds, state->second.bounds);
    }
    if (!Output::neutral(tuple, fun, loc)) {
        auto ret(state->second.elems.emplace_back(std::piecewise_construct, std::forward_as_tuple(tuple), std::forward_as_tuple()));
        auto &elem(ret.first->second);
        if (ret.second || !elem.empty()) {
            if (cond.empty()) {
                elem.clear();
                state->second.accumulate(tuple, fun, true, !ret.second);
            }
            else {
                if (ret.second) { state->second.accumulate(tuple, fun, false, false); }
                elem.emplace_back();
                for (Output::Literal &x : cond) { elem.back().emplace_back(x.clone()); }
            }
            if (state->second.state == BodyAggregateState::DEFINED) {
                state->second._fact = state->second.bounds.contains(state->second.range(fun));
            }
        }
    }
    if (state->second.state == BodyAggregateState::OPEN) {
        exports.append(*state);
        state->second.state = BodyAggregateState::UNKNOWN;
    }
}

void AssignmentAggregateDomain::insert(Value const &repr, ValVec const &tuple, Output::LitVec const &cond, Location const &loc) {
    auto data = dataMap.find(repr);
    if (data == dataMap.end()) { 
        data = dataMap.emplace(std::piecewise_construct, std::forward_as_tuple(repr), std::forward_as_tuple()).first;
        data->second.offset = dataVec.size();
        dataVec.emplace_back(*data);
    }
    if (!Output::neutral(tuple, fun, loc)) {
        auto ret(data->second.elems.emplace_back(std::piecewise_construct, std::forward_as_tuple(tuple), std::forward_as_tuple()));
        auto &elem(ret.first->second);
        if (ret.second || !elem.empty()) {
            if (cond.empty()) { elem.clear(); }
            else {
                elem.emplace_back();
                for (Output::Literal &x : cond) { elem.back().emplace_back(x.clone()); }
            }
        }
    }
    if (data->second.offset < offset) {
        std::swap(dataVec[offset], dataVec[data->second.offset]);
        dataVec[data->second.offset].get().second.offset = data->second.offset;
        data->second.offset = offset;
        --offset;
    }
}

void ConjunctionDomain::insertEmpty() {
    Value repr(rep->eval());
//    std::cerr << "insertEmpty: " << repr << std::endl;
    auto state = domain.find(repr);
    if (state == domain.end()) { 
        state = domain.emplace(std::piecewise_construct, std::forward_as_tuple(repr), std::forward_as_tuple()).first;
        exports.append(*state);
        state->second._generation = 1;
    }
}

void ConjunctionDomain::insert() {
    Value repr(rep->eval());
//    std::cerr << "insert: " << repr << std::endl;
    auto state = domain.find(repr);
    if (state == domain.end()) { 
        state = domain.emplace(std::piecewise_construct, std::forward_as_tuple(repr), std::forward_as_tuple()).first;
    }
    Output::ULitVec cond;
    for (auto &x : lits) { 
        if(auto y = x->toOutput()) { cond.emplace_back(y->clone()); }
    }
    if (head) {
        auto &elem(head->headDom.reserve(head->headRep->eval()));
        if (elem.second.defined()) {
            if (!elem.second.fact(true)) { 
                state->second._fact = false;
                state->second.elems.emplace_back(&elem, std::move(cond));
            }
        }
        else if (cond.empty()) {
            state->second.blocked.emplace(elem.first);
            head->predDom.insert(head->predRep->eval(), true);
//            std::cerr << "  block with: " << elem.first << std::endl;
        }
        else {
            state->second._fact = false;
            state->second.elems.emplace_back(&elem, std::move(cond));
        }
    }
    else if (cond.empty()) { state->second.blocked.emplace(Value()); } // Note: blocks forever
    else {
        state->second._fact = false;
        state->second.elems.emplace_back(nullptr, std::move(cond));
    }
//    std::cerr << "  num blocked: " << state->second.blocked.size() <<  std::endl;
    if (state->second.blocked.empty() && state->second._generation == 0) {
        exports.append(*state);
        state->second._generation = 1;
    }
}
void ConjunctionDomain::unblock(bool fact) {
    Value repr(rep->eval());
    Value hd(head->headRep->eval());
//    std::cerr << "unblock: " << repr << " with " << hd << std::endl;
    auto state = domain.find(repr);
    assert(state != domain.end());
    if (state->second.blocked.erase(hd)) {
        if (!fact) { 
            state->second.elems.emplace_back(&*head->headDom.domain.find(hd), Output::ULitVec());
            state->second._fact = false;
        }
    }
//    std::cerr << "  num blocked: " << state->second.blocked.size() <<  std::endl;
    if (state->second.blocked.empty() && state->second._generation == 0) {
        exports.append(*state);
        state->second._generation = 1;
    }
}

HeadAggregateState &HeadAggregateDomain::insert(Value rep) {
    auto ret(domain.emplace(rep, Output::HeadAggregateState{fun, exports.size()}));
    if (ret.second) {
        Gringo::Ground::init(bounds, ret.first->second.bounds);
        exports.append(*ret.first);
    }
    return ret.first->second;
}
void HeadAggregateDomain::accumulate(unsigned headNum, ValVec const &tuple, Output::LitVec const &lits, Location const &loc) {
    Value rep(this->repr->eval());
    auto &state(domain.find(rep)->second);
    state.accumulate(tuple, fun, headNum > 0 ? &head(headNum).first.reserve(head(headNum).second.repr->eval()) : nullptr, headNum, lits, loc);
    if (!state.todo) {
        state.todo = true;
        todo.emplace_back(state);
    }
}

DisjunctionState &DisjunctionDomain::insert(Value rep) {
    auto ret(domain.emplace(rep, Output::DisjunctionState{exports.size()}));
    if (ret.second) { exports.append(*ret.first); }
    return ret.first->second;
}
void DisjunctionDomain::accumulate(PredicateDomain::element_type *head, Output::LitVec const &lits) {
    Value rep(this->repr->eval());
    auto &state(domain.find(rep)->second);
    state.accumulate(head, lits);
}

void DisjointDomain::insert(Value const &repr, ValVec const &tuple, CSPGroundAdd &&value, int fixed, Output::LitVec const &cond) {
    auto state = domain.find(repr);
    if (state == domain.end()) { 
        state = domain.emplace(std::piecewise_construct, std::forward_as_tuple(repr), std::forward_as_tuple()).first;
        exports.append(*state);
    }
    auto ret(state->second.elems.emplace_back(std::piecewise_construct, std::forward_as_tuple(tuple), std::forward_as_tuple()));
    auto &elem(ret.first->second);
    elem.emplace_back(std::move(value), fixed, Output::ULitVec());
    for (Output::Literal &x : cond) { elem.back().lits.emplace_back(x.clone()); }
}
void DisjointDomain::insert(Value const &repr) {
    auto state = domain.find(repr);
    if (state == domain.end()) { 
        state = domain.emplace(std::piecewise_construct, std::forward_as_tuple(repr), std::forward_as_tuple()).first;
        exports.append(*state);
    }
}

// }}}
// {{{ definition of *Domain::~*Domain

BodyAggregateDomain::~BodyAggregateDomain() { }
AssignmentAggregateDomain::~AssignmentAggregateDomain() { }
ConjunctionDomain::~ConjunctionDomain() { }
HeadAggregateDomain::~HeadAggregateDomain() { }
DisjunctionDomain::~DisjunctionDomain() { }
DisjointDomain::~DisjointDomain() { }

// }}}

// {{{ definition of *BodyOccurrence::getRepr

UGTerm BodyAggregateLiteral::getRepr() const       { return dom->repr->gterm(); }
UGTerm AssignmentAggregateLiteral::getRepr() const { return dom->repr->gterm(); }
UGTerm ConjunctionLiteral::getRepr() const         { return dom->rep->gterm(); }
UGTerm HeadAggregateLiteral::getRepr() const       { return dom->repr->gterm(); }
UGTerm DisjunctionLiteral::getRepr() const         { return dom->repr->gterm(); }
UGTerm DisjointLiteral::getRepr() const            { return dom->repr->gterm(); }

// }}}
// {{{ definition of *BodyOccurrence::isPositive

bool BodyAggregateLiteral::isPositive() const       { return gLit.naf == NAF::POS && dom->positive; }
bool AssignmentAggregateLiteral::isPositive() const { return false; }
bool HeadAggregateLiteral::isPositive() const       { return false; }
bool DisjunctionLiteral::isPositive() const         { return false; }
bool ConjunctionLiteral::isPositive() const         { return true; }
bool DisjointLiteral::isPositive() const            { return false; }

// }}}
// {{{ definition of *BodyOccurrence::isNegative

bool BodyAggregateLiteral::isNegative() const       { return gLit.naf != NAF::POS; }
bool AssignmentAggregateLiteral::isNegative() const { return false; }
bool HeadAggregateLiteral::isNegative() const       { return false; }
bool DisjunctionLiteral::isNegative() const         { return false; }
bool ConjunctionLiteral::isNegative() const         { return false; }
bool DisjointLiteral::isNegative() const            { return gLit.naf != NAF::POS; }

// }}}
// {{{ definition of *BodyOccurrence::setType

void BodyAggregateLiteral::setType(OccurrenceType x)       { type = x; }
void AssignmentAggregateLiteral::setType(OccurrenceType x) { type = x; }
void ConjunctionLiteral::setType(OccurrenceType x)         { type = x; }
void HeadAggregateLiteral::setType(OccurrenceType x)       { type = x; }
void DisjunctionLiteral::setType(OccurrenceType x)         { type = x; }
void DisjointLiteral::setType(OccurrenceType x)            { type = x; }

// }}}
// {{{ definition of *BodyOccurrence::getType

OccurrenceType BodyAggregateLiteral::getType() const       { return type; }
OccurrenceType AssignmentAggregateLiteral::getType() const { return type; }
OccurrenceType ConjunctionLiteral::getType() const         { return type; }
OccurrenceType HeadAggregateLiteral::getType() const       { return type; }
OccurrenceType DisjunctionLiteral::getType() const         { return type; }
OccurrenceType DisjointLiteral::getType() const            { return type; }

// }}}
// {{{ definition of *BodyOccurrence::definedBy

BodyOcc::DefinedBy &BodyAggregateLiteral::definedBy()       { return defs; }
BodyOcc::DefinedBy &AssignmentAggregateLiteral::definedBy() { return defs; }
BodyOcc::DefinedBy &ConjunctionLiteral::definedBy()         { return defs; }
BodyOcc::DefinedBy &HeadAggregateLiteral::definedBy()       { return defs; }
BodyOcc::DefinedBy &DisjunctionLiteral::definedBy()         { return defs; }
BodyOcc::DefinedBy &DisjointLiteral::definedBy()            { return defs; }

// }}}
// {{{ definition of *BodyOccurrence::checkDefined

void BodyAggregateLiteral::checkDefined(LocSet &, SigSet const &) const       { }
void AssignmentAggregateLiteral::checkDefined(LocSet &, SigSet const &) const { }
void ConjunctionLiteral::checkDefined(LocSet &, SigSet const &) const         { }
void HeadAggregateLiteral::checkDefined(LocSet &, SigSet const &) const       { }
void DisjunctionLiteral::checkDefined(LocSet &, SigSet const &) const         { }
void DisjointLiteral::checkDefined(LocSet &, SigSet const &) const            { }

// }}}

// {{{ definition of *Literal::*Literal

BodyAggregateLiteral::BodyAggregateLiteral(SBodyAggregateDomain dom, NAF naf) : dom(dom), gLit(dom->loc)          { gLit.naf = naf; }
AssignmentAggregateLiteral::AssignmentAggregateLiteral(SAssignmentAggregateDomain dom) : dom(dom), gLit(dom->loc) { }
ConjunctionLiteral::ConjunctionLiteral(SConjunctionDomain dom) : dom(dom)                                         { }
HeadAggregateLiteral::HeadAggregateLiteral(SHeadAggregateDomain dom) : dom(dom)                                   { }
DisjunctionLiteral::DisjunctionLiteral(SDisjunctionDomain dom) : dom(dom)                                         { }
DisjointLiteral::DisjointLiteral(SDisjointDomain dom, NAF naf) : dom(dom), gLit(naf)                              { }

// }}}
// {{{ definition of *Literal::print

void BodyAggregateLiteral::print(std::ostream &out) const       { out << gLit.naf << *dom->repr; }
void AssignmentAggregateLiteral::print(std::ostream &out) const { out << *dom->repr; }
void ConjunctionLiteral::print(std::ostream &out) const         { out << *dom->rep; }
void HeadAggregateLiteral::print(std::ostream &out) const       { out << *dom->repr; }
void DisjunctionLiteral::print(std::ostream &out) const         { out << *dom->repr; }
void DisjointLiteral::print(std::ostream &out) const            { out << gLit.naf << *dom->repr; }

// }}}
// {{{ definition of *Literal::isRecursive

bool BodyAggregateLiteral::isRecursive() const       { return type == OccurrenceType::UNSTRATIFIED; }
bool AssignmentAggregateLiteral::isRecursive() const { return type == OccurrenceType::UNSTRATIFIED; }
bool ConjunctionLiteral::isRecursive() const         { return type == OccurrenceType::UNSTRATIFIED; }
bool HeadAggregateLiteral::isRecursive() const       { return type == OccurrenceType::UNSTRATIFIED; }
bool DisjunctionLiteral::isRecursive() const         { return type == OccurrenceType::UNSTRATIFIED; }
bool DisjointLiteral::isRecursive() const            { return type == OccurrenceType::UNSTRATIFIED; }

// }}}
// {{{ definition of *Literal::occurrence

BodyOcc *BodyAggregateLiteral::occurrence()       { return this; }
BodyOcc *AssignmentAggregateLiteral::occurrence() { return this; }
BodyOcc *ConjunctionLiteral::occurrence()         { return this; }
BodyOcc *HeadAggregateLiteral::occurrence()       { return this; }
BodyOcc *DisjunctionLiteral::occurrence()         { return this; }
BodyOcc *DisjointLiteral::occurrence()            { return this; }

// }}}
// {{{ definition of *Literal::collect

void BodyAggregateLiteral::collect(VarTermBoundVec &vars) const       { dom->repr->collect(vars, gLit.naf == NAF::POS); }
void AssignmentAggregateLiteral::collect(VarTermBoundVec &vars) const { dom->repr->collect(vars, true); }
void ConjunctionLiteral::collect(VarTermBoundVec &vars) const         { dom->rep->collect(vars, true); }
void HeadAggregateLiteral::collect(VarTermBoundVec &vars) const       { dom->repr->collect(vars, true); }
void DisjunctionLiteral::collect(VarTermBoundVec &vars) const         { dom->repr->collect(vars, true); }
void DisjointLiteral::collect(VarTermBoundVec &vars) const            { dom->repr->collect(vars, gLit.naf == NAF::POS); }

// }}}
// {{{ definition of *Literal::index

UIdx AssignmentAggregateLiteral::index(Scripts &, BinderType type, Term::VarSet &bound) {
    return make_binder(*dom, NAF::POS, *dom->repr, gLit.repr, type, isRecursive(), bound, 0);
}
UIdx BodyAggregateLiteral::index(Scripts &, BinderType type, Term::VarSet &bound) {
    return make_binder(*dom, gLit.naf, *dom->repr, gLit.repr, type, isRecursive(), bound, 0);
}
UIdx ConjunctionLiteral::index(Scripts &, BinderType type, Term::VarSet &bound) {
    return make_binder(*dom, NAF::POS, *dom->rep, gLit.repr, type, isRecursive(), bound, 0);
}
UIdx HeadAggregateLiteral::index(Scripts &, BinderType type, Term::VarSet &bound) {
    return make_binder(*dom, NAF::POS, *dom->repr, gResult, type, isRecursive(), bound, 0);
}
UIdx DisjunctionLiteral::index(Scripts &, BinderType type, Term::VarSet &bound) {
    return make_binder(*dom, NAF::POS, *dom->repr, gResult, type, isRecursive(), bound, 0);
}
UIdx DisjointLiteral::index(Scripts &, BinderType type, Term::VarSet &bound) {
    return make_binder(*dom, NAF::POS, *dom->repr, gLit.repr, type, isRecursive(), bound, 0);
}

// }}}
// {{{ definition of *Literal::score

Literal::Score BodyAggregateLiteral::score(Term::VarSet const &bound)       { return gLit.naf == NAF::POS ? estimate(dom->exports.size(), *dom->repr, bound) : 0; }
Literal::Score AssignmentAggregateLiteral::score(Term::VarSet const &bound) { return estimate(dom->exports.size(), *dom->repr, bound); }
Literal::Score ConjunctionLiteral::score(Term::VarSet const &bound)         { return estimate(dom->exports.size(), *dom->rep, bound); }
Literal::Score HeadAggregateLiteral::score(Term::VarSet const &bound)       { return estimate(dom->exports.size(), *dom->repr, bound); }
Literal::Score DisjunctionLiteral::score(Term::VarSet const &bound)         { return estimate(dom->exports.size(), *dom->repr, bound); }
Literal::Score DisjointLiteral::score(Term::VarSet const &bound)            { return gLit.naf == NAF::POS ? estimate(dom->exports.size(), *dom->repr, bound) : 0; }

// }}}
// {{{ definition of *Literal::toOutput

Output::Literal *BodyAggregateLiteral::toOutput() {
    gLit.incomplete = isRecursive();
    gLit.fun        = dom->fun;
    gLit.bounds.clear();
    for (auto &x : dom->bounds) { gLit.bounds.emplace_back(x.rel, x.bound->eval()); }
    switch (gLit.naf) {
        case NAF::POS:
        case NAF::NOTNOT: { return !gLit.repr->second.fact(gLit.incomplete) ? &gLit : nullptr; }
        case NAF::NOT:    { return gLit.incomplete || !gLit.repr->second.isFalse() ? &gLit : nullptr; }
    }
    assert(false);
    return nullptr;
}

Output::Literal *AssignmentAggregateLiteral::toOutput() {
    gLit.incomplete = isRecursive();
    gLit.fun        = dom->fun;
    return !gLit.repr->second.fact(gLit.incomplete) ? &gLit : nullptr;
}

Output::Literal *ConjunctionLiteral::toOutput() {
    gLit.incomplete = false;
    for (auto &x : dom->lits) {
        if (x->isRecursive()) { gLit.incomplete = true; }
    }
    return !gLit.repr->second.fact(gLit.incomplete) ? &gLit : nullptr;
}
Output::Literal *HeadAggregateLiteral::toOutput() { return nullptr; }
Output::Literal *DisjunctionLiteral::toOutput()   { return nullptr; }
Output::Literal *DisjointLiteral::toOutput() { 
    gLit.incomplete = isRecursive();
    return &gLit;
}

// }}}
// {{{ definition of *Literal::~*Literal

BodyAggregateLiteral::~BodyAggregateLiteral()             { }
AssignmentAggregateLiteral::~AssignmentAggregateLiteral() { }
ConjunctionLiteral::~ConjunctionLiteral()                 { }
HeadAggregateLiteral::~HeadAggregateLiteral()             { }
DisjunctionLiteral::~DisjunctionLiteral()                 { }
DisjointLiteral::~DisjointLiteral()                       { }

// }}}

// {{{ definition of SolutionCallback::report

void ConjunctionAccumulateFact::report(Output::OutputBase &) {
    bool fact = true;
    Output::LitVec cond;
    for (auto &x : auxLits) { 
        if (x->toOutput()) { 
            fact = false;
            break;
        }
    }
    conjDom->unblock(fact);
}
void ConjunctionAccumulate::report(Output::OutputBase &) {
    conjDom->insert();
}
void ConjunctionAccumulateEmpty::report(Output::OutputBase &) {
    conjDom->insertEmpty();
}
void BodyAggregateAccumulate::report(Output::OutputBase &out) {
    out.tempVals.clear();
    for (auto &x : tuple) { out.tempVals.emplace_back(x->eval()); }
    out.tempLits.clear();
    for (auto &x : lits) {
        if (auto lit = x->toOutput()) { out.tempLits.emplace_back(*lit); }
    }
    dom->insert(defines.repr->eval(), out.tempVals, out.tempLits, tuple.empty() ? dom->repr->loc() : tuple.front()->loc());
}
void DisjointAccumulate::report(Output::OutputBase &out) {
    if (val) {
        out.tempVals.clear();
        for (auto &x : val->tuple) { out.tempVals.emplace_back(x->eval()); }
        CSPGroundLit gVal(Relation::EQ, {}, 0);
        val->value.toGround(gVal, false);
        out.tempLits.clear();
        for (auto &x : lits) {
            if (auto lit = x->toOutput()) { out.tempLits.emplace_back(*lit); }
        }
        dom->insert(defines.repr->eval(), out.tempVals, std::move(std::get<1>(gVal)), -std::get<2>(gVal), out.tempLits);
    }
    else {
        assert(lits.empty());
        dom->insert(defines.repr->eval());
    }
}
void AssignmentAggregateAccumulate::report(Output::OutputBase &out) {
    out.tempVals.clear();
    for (auto &x : tuple) { out.tempVals.emplace_back(x->eval()); }
    out.tempLits.clear();
    for (auto &x : lits) {
        if (auto lit = x->toOutput()) { out.tempLits.emplace_back(*lit); }
    }
    dom->insert(dom->specialRepr->eval(), out.tempVals, out.tempLits, tuple.front()->loc());
}
void HeadAggregateAccumulate::report(Output::OutputBase &out) {
    out.tempVals.clear();
    for (auto &x : tuple) { out.tempVals.emplace_back(x->eval()); }
    out.tempLits.clear();
    for (auto &x : lits) {
        if (auto lit = x->toOutput()) { out.tempLits.emplace_back(*lit); }
    }
    dom->accumulate(headNum, out.tempVals, out.tempLits, tuple.empty() ? dom->repr->loc() : tuple.front()->loc());
}
void HeadAggregateRule::report(Output::OutputBase &out) {
    std::unique_ptr<Output::HeadAggregateRule> rule(make_unique<Output::HeadAggregateRule>());
    rule->fun = dom->fun;
    for (auto &x : lits) {
        if (auto lit = x->toOutput()) { rule->body.emplace_back(Output::ULit(lit->clone())); }
    }
    for (auto &x : dom->bounds) { rule->bounds.emplace_back(x.rel, x.bound->eval()); }
    rule->repr = &dom->insert(def.repr->eval());
    out.output(std::move(rule));
}
void DisjunctionAccumulate::report(Output::OutputBase &out) {
    out.tempLits.clear();
    for (auto &x : lits) {
        if (auto lit = x->toOutput()) { out.tempLits.emplace_back(*lit); }
    }
    PredicateDomain::element_type *head(nullptr);
    if (def) { head = std::get<0>(predDom->insert(def->repr->eval(), false)); }
    dom->accumulate(head, out.tempLits);
}
void DisjunctionRule::report(Output::OutputBase &out) {
    std::unique_ptr<Output::DisjunctionRule> rule(make_unique<Output::DisjunctionRule>());
    for (auto &x : lits) {
        if (auto lit = x->toOutput()) { rule->body.emplace_back(Output::ULit(lit->clone())); }
    }
    rule->repr = &dom->insert(def.repr->eval());
    out.output(std::move(rule));
}
void Rule::report(Output::OutputBase &out) {
    if (external) {
        if (defines) {
            Value val(defines->repr->eval());
            auto ret(domain->insert(val, false));
            out.external(*std::get<0>(ret), Output::ExternalType::E_FALSE);
        }
    }
    else {
        Output::RuleRef &rule(out.tempRule);
        rule.body.clear();
        for (auto &x : lits) {
            if (auto lit = x->toOutput()) { rule.body.emplace_back(*lit); }
        }
        if (defines) {
            auto ret(domain->insert(defines->repr->eval(), rule.body.empty()));
            if (!std::get<2>(ret)) {
                rule.head = std::get<0>(ret);
                out.output(rule);
            }
        }
        else {
            rule.head = nullptr;
            out.output(rule);
        }
    }
}
void WeakConstraint::report(Output::OutputBase &out) {
    out.tempVals.clear();
    for (auto &x : tuple) { out.tempVals.emplace_back(x->eval()); }
    if (out.tempVals.front().type() == Value::NUM) {
        Output::ULitVec cond;
        for (auto &x : lits) {
            if (auto lit = x->toOutput()) { cond.emplace_back(lit->clone()); }
        }
        Output::Minimize min;
        min.elems.emplace_back(
            std::piecewise_construct, 
            std::forward_as_tuple(out.tempVals), 
            std::forward_as_tuple(std::move(cond)));
        out.output(min);
    }
    else {
        GRINGO_REPORT(W_TERM_UNDEFINED) 
            << tuple.front()->loc() << ": warning: weak constraint not defined for weight, tuple is ignored:\n"
            << "  " << out.tempVals.front() << "\n";
    }
}

// }}}
// {{{ definition of SolutionCallback::mark

void ConjunctionAccumulateFact::mark() { conjDom->mark(); }
void ConjunctionAccumulate::mark() {
    conjDom->mark();
    if (conjDom->head) { conjDom->head->predDom.mark(); }
}
void ConjunctionAccumulateEmpty::mark()    { conjDom->mark(); }
void BodyAggregateAccumulate::mark()       { dom->mark(); }
void DisjointAccumulate::mark()            { dom->mark(); }
void AssignmentAggregateAccumulate::mark() { dom->mark(); }
void HeadAggregateAccumulate::mark()       { dom->accumulateMark(); }
void HeadAggregateRule::mark()             { dom->mark(); }
void DisjunctionRule::mark()               { dom->mark(); }
void DisjunctionAccumulate::mark()         { if (def) { predDom->mark(); } }
void Rule::mark()                          { if (defines) { domain->mark(); } }
void WeakConstraint::mark()                { }

// }}}
// {{{ definition of SolutionCallback::unmark

void ConjunctionAccumulateFact::unmark(Queue &queue) {
    conjDom->unmark();
    conjDef.enqueue(queue);
    queue.enqueue(*conjDom);
}
void ConjunctionAccumulate::unmark(Queue &queue) {
    conjDom->unmark();
    conjDef.enqueue(queue);
    queue.enqueue(*conjDom);
    if (conjDom->head) { 
        conjDom->head->predDom.unmark();
        conjDom->head->predDef.enqueue(queue);
        queue.enqueue(conjDom->head->predDom);
    }
}
void ConjunctionAccumulateEmpty::unmark(Queue &queue) {
    conjDom->unmark();
    conjDef.enqueue(queue);
    queue.enqueue(*conjDom);
}
void BodyAggregateAccumulate::unmark(Queue &queue) {
    dom->unmark();
    defines.enqueue(queue);
    queue.enqueue(*dom);
}
void DisjointAccumulate::unmark(Queue &queue) {
    dom->unmark();
    defines.enqueue(queue);
    queue.enqueue(*dom);
}
void AssignmentAggregateAccumulate::unmark(Queue &queue) {
    dom->unmark();
    defines.enqueue(queue);
    queue.enqueue(*dom);
}
void HeadAggregateAccumulate::unmark(Queue &queue) {
    dom->accumulateUnmark(queue);
}
void HeadAggregateRule::unmark(Queue &queue) {
    dom->unmark();
    def.enqueue(queue);
    queue.enqueue(*dom);
}
void DisjunctionAccumulate::unmark(Queue &queue) {
    if (def) { 
        predDom->unmark();
        def->enqueue(queue);
        queue.enqueue(*predDom);
    }
}
void DisjunctionRule::unmark(Queue &queue) {
    dom->unmark();
    def.enqueue(queue);
    queue.enqueue(*dom);
}
void Rule::unmark(Queue &queue) {
    if (defines) { 
        domain->unmark();
        defines->enqueue(queue);
        queue.enqueue(*domain);
    }
}
void WeakConstraint::unmark(Queue &) { }

// }}}
// {{{ definition of SolutionCallback::printHead

void ConjunctionAccumulateFact::printHead(std::ostream &out) const {
    out << "#fact(" << *conjDom->rep << ")";
}
void ConjunctionAccumulate::printHead(std::ostream &out) const {
    if (conjDom->head) { out << *conjDom->head->predRep; }
    else               { out << "false"; }
    out << ":" << *conjDom->rep;
}
void ConjunctionAccumulateEmpty::printHead(std::ostream &out) const {
    out << "#true:" << *conjDom->rep;
}
void BodyAggregateAccumulate::printHead(std::ostream &out) const {
    auto f = [](std::ostream &out, UTerm const &x) { out << *x; };
    print_comma(out, tuple, ",", f);
    out << ":" << *defines.repr;
}
void DisjointAccumulate::printHead(std::ostream &out) const {
    auto f = [](std::ostream &out, UTerm const &x) { out << *x; };
    if (val) {
        print_comma(out, val->tuple, ",", f);
        out << ":" << val->value;
    }
    else { out << "#empty"; }
    out << ":" << *defines.repr;
}
void AssignmentAggregateAccumulate::printHead(std::ostream &out) const {
    auto f = [](std::ostream &out, UTerm const &x) { out << *x; };
    print_comma(out, tuple, ",", f);
    out << ":" << *defines.repr;
}
void HeadAggregateAccumulate::printHead(std::ostream &out) const {
    auto f = [](std::ostream &out, UTerm const &x) { out << *x; };
    print_comma(out, tuple, ",", f);
    out << ":";
    if (headNum > 0) { out << *dom->head(headNum).second.repr; }
    else             { out << "#true"; }
}
void HeadAggregateRule::printHead(std::ostream &out) const {
    out << *def.repr;
}
void DisjunctionAccumulate::printHead(std::ostream &out) const {
    if (def) { out << *def->repr; }
    else     { out << "#true"; }
}
void DisjunctionRule::printHead(std::ostream &out) const {
    out << *def.repr;
}
void Rule::printHead(std::ostream &out) const {
    if (defines) { out << *defines->repr; }
}
void WeakConstraint::printHead(std::ostream &out) const {
    out << "[";
    out << *tuple.front() << "@" << *tuple[1];
    for (auto it(tuple.begin() + 2), ie(tuple.end()); it != ie; ++it) { out << "," << **it; }
    out << "]";
}

// }}}

// {{{ definition of *Statement::*Statement

BodyAggregateAccumulate::BodyAggregateAccumulate(SBodyAggregateDomain dom, UTermVec &&tuple, ULitVec &&lits, ULitVec &&auxLits)
    : dom(dom)
    , defines(get_clone(dom->repr))
    , tuple(std::move(tuple))
    , lits(std::move(lits))
    , auxLits(std::move(auxLits)) { dom->numBlocked++; }

DisjointAccumulate::DisjointAccumulate(SDisjointDomain dom, UTermVec &&tuple, CSPAddTerm &&value, ULitVec &&lits, ULitVec &&auxLits)
    : dom(dom)
    , defines(get_clone(dom->repr))
    , val(make_unique<Value>(std::move(tuple), std::move(value)))
    , lits(std::move(lits))
    , auxLits(std::move(auxLits)) { dom->numBlocked++; }
DisjointAccumulate::DisjointAccumulate(SDisjointDomain dom, ULitVec &&lits, ULitVec &&auxLits)
    : dom(dom)
    , defines(get_clone(dom->repr))
    , val(nullptr)
    , lits(std::move(lits))
    , auxLits(std::move(auxLits)) { dom->numBlocked++; }

AssignmentAggregateAccumulate::AssignmentAggregateAccumulate(SAssignmentAggregateDomain dom, UTermVec const &tuple, ULitVec &&lits, ULitVec &&auxLits)
    : dom(dom)
    , defines(UTerm(dom->repr->clone()))
    , tuple(get_clone(tuple))
    , lits(std::move(lits))
    , auxLits(std::move(auxLits)) { dom->numBlocked++; }

ConjunctionAccumulateEmpty::ConjunctionAccumulateEmpty(SConjunctionDomain conjDom, ULitVec &&auxLits)
    : conjDom(conjDom)
    , conjDef(get_clone(conjDom->rep))
    , auxLits(std::move(auxLits)) { conjDom->numBlocked++; }

ConjunctionAccumulate::ConjunctionAccumulate(SConjunctionDomain conjDom, ULitVec &&auxLits)
    : conjDom(conjDom)
    , conjDef(get_clone(conjDom->rep))
    , auxLits(std::move(auxLits)) { conjDom->numBlocked++; }

ConjunctionAccumulateFact::ConjunctionAccumulateFact(SConjunctionDomain conjDom, ULitVec &&auxLits)
    : conjDom(conjDom)
    , conjDef(get_clone(conjDom->rep))
    , auxLits(std::move(auxLits)) { conjDom->numBlocked++; }

HeadAggregateAccumulate::HeadAggregateAccumulate(SHeadAggregateDomain dom, UTermVec &&tuple, PredicateDomain *headDom, UTerm &&repr, ULitVec &&lits)
    : dom(dom)
    , tuple(std::move(tuple))
    , headNum(0)
    , lits(std::move(lits)) { 
    dom->numBlocked++; 
    if (headDom) {
        dom->heads.emplace_back(*headDom, std::move(repr));
        headNum = dom->heads.size();
    }
}

DisjunctionAccumulate::DisjunctionAccumulate(SDisjunctionDomain dom, PredicateDomain *predDom, UTerm &&repr, ULitVec &&lits)
    : dom(dom)
    , predDom(predDom)
    , def(predDom ? make_unique<HeadDefinition>(std::move(repr)) : nullptr)
    , lits(std::move(lits)) { }

HeadAggregateRule::HeadAggregateRule(SHeadAggregateDomain dom, ULitVec &&lits)
    : dom(dom)
    , def(get_clone(dom->repr))
    , lits(std::move(lits)) { }

DisjunctionRule::DisjunctionRule(SDisjunctionDomain dom, ULitVec &&lits) 
    : dom(dom)
    , def(get_clone(dom->repr))
    , lits(std::move(lits)) { }

Rule::Rule(PredicateDomain *domain, UTerm &&repr, ULitVec &&lits, bool external)
    : domain(domain)
    , defines(repr ? make_unique<HeadDefinition>(std::move(repr)) : nullptr)
    , lits(std::move(lits))
    , external(external) { }

ExternalRule::ExternalRule()
    : defines(make_locatable<ValTerm>(Location("#external", 1, 1, "#external", 1, 1), "#external")) { }

WeakConstraint::WeakConstraint(UTermVec &&tuple, ULitVec &&lits)
    : tuple(std::move(tuple))
    , lits(std::move(lits)) { }

// }}}
// {{{ definition of *Statement::isNormal

bool BodyAggregateAccumulate::isNormal() const       { return true; }
bool DisjointAccumulate::isNormal() const            { return true; }
bool AssignmentAggregateAccumulate::isNormal() const { return true; }
bool ConjunctionAccumulateEmpty::isNormal() const    { return true; }
bool ConjunctionAccumulate::isNormal() const         { return true; }
bool ConjunctionAccumulateFact::isNormal() const     { return true; }
bool HeadAggregateAccumulate::isNormal() const       { return true; }
bool DisjunctionAccumulate::isNormal() const         { return true; }
bool HeadAggregateRule::isNormal() const             { return false; }
bool DisjunctionRule::isNormal() const               { return false; }
bool Rule::isNormal() const                          { return bool(defines) && !external; }
bool WeakConstraint::isNormal() const                { return true; }
bool ExternalRule::isNormal() const                  { return false; }

// }}}
// {{{ definition of *Statement::analyze

namespace {

void _analyze(Statement::Dep::Node &node, Statement::Dep &dep, ULitVec &lits, bool forceNegative = false) {
    for (auto &x : lits) {
        auto occ(x->occurrence());
        if (occ) { dep.depends(node, *occ, forceNegative); }
    }
}
void _analyze(Statement::Dep::Node &node, Statement::Dep &dep, ULitVec &a, ULitVec &b) {
    _analyze(node, dep, a);
    _analyze(node, dep, b);
}
    
} // namespace

void BodyAggregateAccumulate::analyze(Dep::Node &node, Dep &dep) {
    dep.provides(node, defines, defines.getRepr());
    _analyze(node, dep, lits, auxLits);
}
void DisjointAccumulate::analyze(Dep::Node &node, Dep &dep) {
    dep.provides(node, defines, defines.getRepr());
    _analyze(node, dep, lits, auxLits);
    dep.depends(node, ext);
}
void AssignmentAggregateAccumulate::analyze(Dep::Node &node, Dep &dep) {
    dep.provides(node, defines, dom->repr->gterm());
    _analyze(node, dep, lits, auxLits);
}
void ConjunctionAccumulateEmpty::analyze(Dep::Node &node, Dep &dep) {
    dep.provides(node, conjDef, conjDom->rep->gterm());
    _analyze(node, dep, auxLits);
}
void ConjunctionAccumulate::analyze(Dep::Node &node, Dep &dep) {
    dep.provides(node, conjDef, conjDom->rep->gterm());
    if (conjDom->head) {
        dep.provides(node, conjDom->head->predDef, conjDom->head->predRep->gterm());
        dep.depends(node, *conjDom->head);
    }
    _analyze(node, dep, conjDom->lits, true);
    _analyze(node, dep, auxLits);
}
void ConjunctionAccumulateFact::analyze(Dep::Node &node, Dep &dep) {
    dep.provides(node, conjDef, conjDom->rep->gterm());
    _analyze(node, dep, auxLits);
}
void HeadAggregateAccumulate::analyze(Dep::Node &node, Dep &dep) {
    if (headNum > 0) { dep.provides(node, dom->head(headNum).second, dom->head(headNum).second.getRepr()); }
    // TODO: find out why exactly I added this dependency
    dep.provides(node, dom->dummy, dom->dummy.getRepr());
    dep.depends(node, dom->dummy);
    _analyze(node, dep, lits);
}
void DisjunctionAccumulate::analyze(Dep::Node &node, Dep &dep) {
    if (def) { dep.provides(node, *def, def->getRepr()); }
    _analyze(node, dep, lits);
}
void HeadAggregateRule::analyze(Dep::Node &node, Dep &dep) {
    dep.provides(node, def, def.getRepr());
    _analyze(node, dep, lits);
}
void DisjunctionRule::analyze(Dep::Node &node, Dep &dep) {
    dep.provides(node, def, def.getRepr());
    _analyze(node, dep, lits);
}
void Rule::analyze(Dep::Node &node, Dep &dep) {
    if (defines) { dep.provides(node, *defines, defines->getRepr()); }
    _analyze(node, dep, lits);
}
void WeakConstraint::analyze(Dep::Node &node, Dep &dep) {
    _analyze(node, dep, lits);
}
void ExternalRule::analyze(Dep::Node &node, Dep &dep) {
    dep.provides(node, defines, defines.getRepr());
}

// }}}
// {{{ definition of *Statement::startLinearize

void BodyAggregateAccumulate::startLinearize(bool active) {
    defines.active = active;
    if (active)  { insts.clear(); }
}
void DisjointAccumulate::startLinearize(bool active) {
    defines.active = active;
    if (active)  { insts.clear(); }
}
void AssignmentAggregateAccumulate::startLinearize(bool active) {
    defines.active = active;
    if (active)  { insts.clear(); }
}
void ConjunctionAccumulateEmpty::startLinearize(bool active) {
    conjDef.active = active;
    if (active)  { insts.clear(); }
}
void ConjunctionAccumulate::startLinearize(bool active) {
    conjDef.active = active;
    if (conjDom->head) { conjDom->head->predDef.active = active; }
    if (active)  { insts.clear(); }
}
void HeadAggregateAccumulate::startLinearize(bool active) {
    if (headNum > 0) { dom->head(headNum).second.active = active; }
    if (active)  { insts.clear(); }
}
void HeadAggregateRule::startLinearize(bool active) {
    def.active = active;
    if (active)  { insts.clear(); }
}
void DisjunctionAccumulate::startLinearize(bool active) {
    if (def) { def->active = active; }
    if (active)  { insts.clear(); }
}
void DisjunctionRule::startLinearize(bool active) {
    def.active = active;
    if (active)  { insts.clear(); }
}
void ConjunctionAccumulateFact::startLinearize(bool active) {
    conjDef.active = active;
    if (active)  { insts.clear(); }
}
void Rule::startLinearize(bool active) {
    if (defines) { defines->active = active; }
    if (active)  { insts.clear(); }
}
void WeakConstraint::startLinearize(bool active) {
    if (active)  { insts.clear(); }
}
void ExternalRule::startLinearize(bool active) {
    defines.active = active;
}

// }}}
// {{{ definition of *Statement::linearize

namespace {

    struct Ent {
        Ent(BinderType type, Literal &lit) : type(type), lit(lit) { }
        std::vector<unsigned> depends;
        std::vector<unsigned> vars;
        BinderType type;
        Literal &lit;
    };
    using SC  = SafetyChecker<unsigned, Ent>;

    InstVec linearize(Scripts &scripts, bool positive, SolutionCallback &cb, Term::VarSet &&important, ULitVec &lits, ULitVec *aux = nullptr) {
        InstVec insts;
        std::vector<unsigned> rec;
        std::vector<std::vector<std::pair<BinderType,Literal*>>> todo{1};
        unsigned i{0};
        for (auto &x : lits) {
            todo.back().emplace_back(BinderType::ALL, x.get());
            if (x->isRecursive() && x->occurrence() && !x->occurrence()->isNegative()) { rec.emplace_back(i); }
            ++i;
        }
        if (aux) {
            for (auto &x : *aux) { 
                todo.back().emplace_back(BinderType::ALL, x.get());
                if (x->isRecursive() && x->occurrence() && !x->occurrence()->isNegative()) { rec.emplace_back(i); }
                ++i;
            }
        }
        todo.reserve(std::max(std::vector<unsigned>::size_type(1), rec.size()));
        insts.reserve(todo.capacity()); // Note: preserve references
        for (auto i : rec) {
            todo.back()[i].first = BinderType::NEW;
            if (i != rec.back()) { 
                todo.emplace_back(todo.back());
                todo.back()[i].first = BinderType::OLD;
            }
        }
        if (!positive) {
            for (auto &lit : lits) { lit->collectImportant(important); }
        }
        for (auto &x : todo) {
            insts.emplace_back(cb);
            SC s;
            std::unordered_map<FWString, SC::VarNode*> varMap;
            std::vector<std::pair<FWString, std::vector<unsigned>>> boundBy;
            for (auto &lit : x) {
                auto &entNode(s.insertEnt(lit.first, *lit.second));
                VarTermBoundVec vars;
                lit.second->collect(vars);
                for (auto &occ : vars) {
                    auto &varNode(varMap[occ.first->name]);
                    if (!varNode)   { 
                        varNode = &s.insertVar(boundBy.size());
                        boundBy.emplace_back(occ.first->name, std::vector<unsigned>{});
                    }
                    if (occ.second) { s.insertEdge(entNode, *varNode); }
                    else            { s.insertEdge(*varNode, entNode); }
                    entNode.data.vars.emplace_back(varNode->data);
                }
            }
            Term::VarSet bound;
            Instantiator::DependVec depend;
            unsigned uid = 0;
            auto pred = [&bound](Ent const &x, Ent const &y) -> bool {
                double sx(x.lit.score(bound));
                double sy(y.lit.score(bound));
                //std::cerr << "  " << x.lit << "@" << sx << " < " << y.lit << "@" << sy << " with " << bound.size() << std::endl;
                if (sx < 0 || sy < 0) { return sx < sy; }
                if ((x.type == BinderType::NEW || y.type == BinderType::NEW) && x.type != y.type) { return x.type < y.type; }
                return sx < sy;
            };

            SC::EntVec open;
            s.init(open);
            while (!open.empty()) {
                for (auto it = open.begin(), end = open.end() - 1; it != end; ++it) {
                    if (pred((*it)->data, open.back()->data)) { std::swap(open.back(), *it); }
                }
                auto y = open.back();
                for (auto &var : y->data.vars) {
                    auto &bb(boundBy[var]);
                    if (bound.find(bb.first) == bound.end()) {
                        bb.second.emplace_back(uid);
                        if ((depend.empty() || depend.back() != uid) && important.find(bb.first) != important.end()) { depend.emplace_back(uid); }
                    }
                    else { y->data.depends.insert(y->data.depends.end(), bb.second.begin(), bb.second.end()); }
                }
                auto index(y->data.lit.index(scripts, y->data.type, bound));
                if (auto update = index->getUpdater()) {
                    if (BodyOcc *occ = y->data.lit.occurrence()) {
                        for (HeadOccurrence &x : occ->definedBy()) { x.defines(*update, y->data.type == BinderType::NEW ? &insts.back() : nullptr); }
                    }
                }
                std::sort(y->data.depends.begin(), y->data.depends.end());
                y->data.depends.erase(std::unique(y->data.depends.begin(), y->data.depends.end()), y->data.depends.end());
                insts.back().add(std::move(index), std::move(y->data.depends));
                uid++;
                open.pop_back();
                s.propagate(y, open);
            }
            insts.back().finalize(std::move(depend));
        }
        return insts;
    }

} // namespace

void BodyAggregateAccumulate::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    defines.collectImportant(important);
    VarTermBoundVec vars;
    for (auto &x : tuple) { x->collect(vars, false); }
    for (auto &x : vars)  { important.emplace(x.first->name); }
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits, &auxLits);
}
void DisjointAccumulate::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    defines.collectImportant(important);
    VarTermBoundVec vars;
    if (val) {
        for (auto &x : val->tuple) { x->collect(vars, false); }
        val->value.collect(vars);
    }
    for (auto &x : vars)  { important.emplace(x.first->name); }
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits, &auxLits);
}
void AssignmentAggregateAccumulate::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    defines.collectImportant(important);
    VarTermBoundVec vars;
    for (auto &x : tuple) { x->collect(vars, false); }
    for (auto &x : vars)  { important.emplace(x.first->name); }
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits, &auxLits);
}
void ConjunctionAccumulateEmpty::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    conjDef.collectImportant(important);
    ULitVec lits;
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits, &auxLits);
}
void ConjunctionAccumulate::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    conjDef.collectImportant(important);
    if (conjDom->head) { 
        conjDom->head->predDef.collectImportant(important);
        conjDom->head->headRep->collect(important);
    }
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), conjDom->lits, &auxLits);
}
void ConjunctionAccumulateFact::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    conjDef.collectImportant(important);
    conjDom->head->headRep->collect(important);
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), conjDom->lits, &auxLits);
}
void HeadAggregateAccumulate::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    if (headNum > 0) { dom->head(headNum).second.collectImportant(important); }
    for (auto &x : tuple) { x->collect(important); }
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits);
}
void DisjunctionAccumulate::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    if (def) { def->collectImportant(important); }
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits);
}
void HeadAggregateRule::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    def.collectImportant(important);
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits);
}
void DisjunctionRule::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    def.collectImportant(important);
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits);
}
void Rule::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    if (defines) { defines->collectImportant(important); }
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits);
}
void WeakConstraint::linearize(Scripts &scripts, bool positive) {
    Term::VarSet important;
    for (auto &x : tuple) { x->collect(important); }
    insts = Gringo::Ground::linearize(scripts, positive, *this, std::move(important), lits);
}
void ExternalRule::linearize(Scripts &, bool) { }

// }}}
// {{{ definition of *Statement::enqueue

void ConjunctionAccumulateEmpty::enqueue(Queue &q) {
    if (!conjDom->blocked)   { conjDom->blocked = conjDom->numBlocked; }
    --conjDom->blocked;
    conjDom->init();
    for (auto &x : insts) { x.enqueue(q); }
}
void ConjunctionAccumulate::enqueue(Queue &q) {
    if (!conjDom->blocked)   { conjDom->blocked = conjDom->numBlocked; }
    --conjDom->blocked;
    conjDom->init();
    for (auto &x : insts) { x.enqueue(q); }
}
void ConjunctionAccumulateFact::enqueue(Queue &q) {
    if (!conjDom->blocked)   { conjDom->blocked = conjDom->numBlocked; }
    --conjDom->blocked;
    conjDom->init();
    if (conjDom->head)    { conjDom->head->predDom.init(); }
    for (auto &x : insts) { x.enqueue(q); }
}
void BodyAggregateAccumulate::enqueue(Queue &q) {
    if (!dom->blocked)   { dom->blocked = dom->numBlocked; }
    --dom->blocked;
    dom->init();
    for (auto &x : insts) { x.enqueue(q); }
}
void DisjointAccumulate::enqueue(Queue &q) {
    if (!dom->blocked)   { dom->blocked = dom->numBlocked; }
    --dom->blocked;
    dom->init();
    for (auto &x : insts) { x.enqueue(q); }
}
void AssignmentAggregateAccumulate::enqueue(Queue &q) {
    if (!dom->blocked)   { dom->blocked = dom->numBlocked; }
    --dom->blocked;
    dom->init();
    for (auto &x : insts) { x.enqueue(q); }
}
void HeadAggregateAccumulate::enqueue(Queue &q) {
    if (!dom->blocked)   { dom->blocked = dom->numBlocked; }
    --dom->blocked;
    if (headNum > 0) { dom->head(headNum).first.init(); }
    for (auto &x : insts) { x.enqueue(q); }
}
void HeadAggregateRule::enqueue(Queue &q) {
    dom->init();
    for (auto &x : insts) { x.enqueue(q); }
}
void DisjunctionAccumulate::enqueue(Queue &q) {
    if (def) { predDom->init(); }
    for (auto &x : insts) { x.enqueue(q); }
}
void DisjunctionRule::enqueue(Queue &q) {
    dom->init();
    for (auto &x : insts) { x.enqueue(q); }
}
void Rule::enqueue(Queue &q) {
    if (defines) { domain->init(); }
    for (auto &x : insts) { x.enqueue(q); }
}
void WeakConstraint::enqueue(Queue &q) {
    for (auto &x : insts) { x.enqueue(q); }
}
void ExternalRule::enqueue(Queue &) {
}

// }}}
// {{{ definition of *Statement::print

void BodyAggregateAccumulate::print(std::ostream &out) const {
    auto f = [](std::ostream &out, UTerm const &x) { out << *x; };
    out << "#accumulate(" << *defines.repr << ",tuple(";
    print_comma(out, tuple, ",", f);
    out << ")):-";
    auto g = [](std::ostream &out, ULit const &x) { out << *x; };
    print_comma(out, lits, ",", g);
    out << ":-";
    print_comma(out, auxLits, ",", g);
    out << ".";
}
void DisjointAccumulate::print(std::ostream &out) const {
    auto f = [](std::ostream &out, UTerm const &x) { out << *x; };
    out << "#accumulate(" << *defines.repr;
    if (val) {
        out << ",tuple(";
        print_comma(out, val->tuple, ",", f);
        out << ")" ;
        out << "," << val->value;
    }
    else { out << ",#empty"; }
    out << "):-";
    auto g = [](std::ostream &out, ULit const &x) { out << *x; };
    print_comma(out, lits, ",", g);
    out << ":-";
    print_comma(out, auxLits, ",", g);
    out << ".";
}
void AssignmentAggregateAccumulate::print(std::ostream &out) const {
    auto f = [](std::ostream &out, UTerm const &x) { out << *x; };
    out << "#accumulate(" << *dom->specialRepr << ",tuple(";
    print_comma(out, tuple, ",", f);
    out << ")):-";
    auto g = [](std::ostream &out, ULit const &x) { out << *x; };
    print_comma(out, lits, ",", g);
    out << ":-";
    print_comma(out, auxLits, ",", g);
    out << ".";
}
void ConjunctionAccumulateEmpty::print(std::ostream &out) const {
    out << "#accumulate(" << *conjDom->rep << ",#true)):-:-";
    auto g = [](std::ostream &out, ULit const &x) { out << *x; };
    print_comma(out, auxLits, ",", g);
    out << ".";
}
void ConjunctionAccumulate::print(std::ostream &out) const {
    out << "#accumulate(" << *conjDom->rep << ",atom(";
    if (conjDom->head) { out << *conjDom->head->headRep; }
    else               { out << "#false"; } 
    out << ")):-";
    auto g = [](std::ostream &out, ULit const &x) { out << *x; };
    print_comma(out, conjDom->lits, ",", g);
    out << ":-";
    print_comma(out, auxLits, ",", g);
    out << ".";
}
void ConjunctionAccumulateFact::print(std::ostream &out) const {
    out << "#accumulate(" << *conjDom->rep << ",atom(" << *conjDom->head->headRep << ")):-";
    auto g = [](std::ostream &out, ULit const &x) { out << *x; };
    print_comma(out, auxLits, ",", g);
    out << ".";
}
void HeadAggregateAccumulate::print(std::ostream &out) const {
    auto f = [](std::ostream &out, UTerm const &x) { out << *x; };
    out << "#accumulate(" << *dom->repr << ",tuple(";
    print_comma(out, tuple, ",", f);
    out << "),head(";
    if (headNum > 0) { out << *dom->head(headNum).second.repr; }
    else             { out << "#true"; }
    out << ")):-";
    auto g = [](std::ostream &out, ULit const &x) { out << *x; };
    print_comma(out, lits, ",", g);
    out << ".";
}
void DisjunctionAccumulate::print(std::ostream &out) const {
    if (def) { out << *def->repr; }
    else     { out << "#true"; }
    out << ":-";
    auto g = [](std::ostream &out, ULit const &x) { out << *x; };
    print_comma(out, lits, ",", g);
    out << ".";
}
void HeadAggregateRule::print(std::ostream &out) const {
    auto it = dom->bounds.begin(), ie = dom->bounds.end();
    if (it != ie) {
        out << *it->bound;
        out << inv(it->rel);
        ++it;
    }
    out << dom->fun << "(" << *dom->repr << ")";
    for (; it != ie; ++it) {
        out << it->rel;
        out << *it->bound;
    }
    if (!lits.empty()) {
        out << ":-";
        auto g = [](std::ostream &out, ULit const &x) { if (x) { out << *x; } else { out << "#null?"; }  };
        print_comma(out, lits, ",", g);
    }
    out << ".";
}
void DisjunctionRule::print(std::ostream &out) const {
    out << *dom->repr;
    if (!lits.empty()) {
        out << ":-";
        auto g = [](std::ostream &out, ULit const &x) { out << *x; };
        print_comma(out, lits, ",", g);
    }
    out << ".";
}
void Rule::print(std::ostream &out) const {
    if (external) { out << "#external "; }
    if (defines) { out << *defines->repr; }
    if (!defines || !lits.empty()) { out << ":-"; }
    if (!lits.empty()) {
        auto f = [](std::ostream &out, ULit const &lit) { out << *lit; };
        print_comma(out, lits, ",", f);
    }
    out << ".";
}
void WeakConstraint::print(std::ostream &out) const {
    out << ":~"; 
    print_comma(out, lits, ";", [](std::ostream &out, ULit const &x) { out << *x; });
    out << ".";
    printHead(out);
}
void ExternalRule::print(std::ostream &out) const {
    out << "#external."; 
}

// }}}
// {{{ definition of *Statement::~*Statement

BodyAggregateAccumulate::~BodyAggregateAccumulate()             { }
DisjointAccumulate::~DisjointAccumulate()                       { }
AssignmentAggregateAccumulate::~AssignmentAggregateAccumulate() { }
ConjunctionAccumulateEmpty::~ConjunctionAccumulateEmpty()       { }
ConjunctionAccumulate::~ConjunctionAccumulate()                 { }
ConjunctionAccumulateFact::~ConjunctionAccumulateFact()         { }
HeadAggregateAccumulate::~HeadAggregateAccumulate()             { }
DisjunctionAccumulate::~DisjunctionAccumulate()                 { }
HeadAggregateRule::~HeadAggregateRule()                         { }
DisjunctionRule::~DisjunctionRule()                             { }
Rule::~Rule()                                                   { }
WeakConstraint::~WeakConstraint()                               { }
ExternalRule::~ExternalRule()                                   { }

// }}}

} } // namespace Ground Gringo

