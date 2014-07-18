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

#include "gringo/bug.hh"
#include "gringo/input/aggregates.hh"
#include "gringo/input/literals.hh"
#include "gringo/ground/statements.hh"
#include "gringo/ground/literals.hh"
#include "gringo/logger.hh"

namespace Gringo { namespace Input {

// {{{ definition of Aggregate::print

namespace {

template <class T, class U, class V>
void _print(std::ostream &out, AggregateFunction fun, T const &x, U const &y, V const &f) {
    auto it = std::begin(x), ie = std::end(x); 
    if (it != ie) {
        it->bound->print(out);
        out << inv(it->rel);
        ++it;
    }
    out << fun << "{";
    print_comma(out, y, ";", f);
    out << "}";
    for (; it != ie; ++it) {
        out << it->rel;
        it->bound->print(out);
    }
}

auto _printCond = [](std::ostream &out, CondLit const &x) {
    using namespace std::placeholders;
    x.first->print(out);
    out << ":";
    print_comma(out, x.second, ",", std::bind(&Literal::print, _2, _1));
};

} // namespace

void TupleBodyAggregate::print(std::ostream &out) const {
    auto f = [](std::ostream &out, BodyAggrElem const &y) {
        using namespace std::placeholders;
        print_comma(out, y.first, ",", std::bind(&Term::print, _2, _1));
        out << ":";
        print_comma(out, y.second, ",", std::bind(&Literal::print, _2, _1));
    };
    out << naf;
    _print(out, fun, bounds, elems, f);
}
void LitBodyAggregate::print(std::ostream &out) const {
    out << naf;
    _print(out, fun, bounds, elems, _printCond);
}
void Conjunction::print(std::ostream &out) const { 
    head->print(out);
    out << ":";
    using namespace std::placeholders;
    print_comma(out, cond, ",", std::bind(&Literal::print, _2, _1));
}
void SimpleBodyLiteral::print(std::ostream &out) const { lit->print(out); }
// head
void TupleHeadAggregate::print(std::ostream &out) const {
    auto f = [](std::ostream &out, HeadAggrElem const &y) {
        using namespace std::placeholders;
        print_comma(out, std::get<0>(y), ",", std::bind(&Term::print, _2, _1));
        out << ":";
        std::get<1>(y)->print(out);
        out << ":";
        print_comma(out, std::get<2>(y), ",", std::bind(&Literal::print, _2, _1));
    };
    _print(out, fun, bounds, elems, f);
}
void LitHeadAggregate::print(std::ostream &out) const {
    _print(out, fun, bounds, elems, _printCond);
}
void Disjunction::print(std::ostream &out) const { 
    auto f = [](std::ostream &out, CondLit const &y) {
        using namespace std::placeholders;
        y.first->print(out);
        out << ":";
        print_comma(out, y.second, ",", std::bind(&Literal::print, _2, _1));
    };
    print_comma(out, elems, ";", f);
}
void SimpleHeadLiteral::print(std::ostream &out) const { lit->print(out); }

void DisjointAggregate::print(std::ostream &out) const {
    out << naf << "#disjoint{";
    print_comma(out, elems, ";");
    out << "}";
}

// }}}
// {{{ definition of Aggregate::hash

size_t TupleBodyAggregate::hash() const {
    return get_value_hash(typeid(TupleBodyAggregate).hash_code(), size_t(naf), size_t(fun), bounds, elems);
}
size_t LitBodyAggregate::hash() const {
    return get_value_hash(typeid(LitBodyAggregate).hash_code(), size_t(naf), size_t(fun), bounds, elems);
}
size_t Conjunction::hash() const { 
    return get_value_hash(typeid(Conjunction).hash_code(), head, cond);
}
size_t SimpleBodyLiteral::hash() const { 
    return get_value_hash(typeid(SimpleBodyLiteral).hash_code(), lit);
}
size_t TupleHeadAggregate::hash() const {
    return get_value_hash(typeid(TupleHeadAggregate).hash_code(), size_t(fun), bounds, elems);
}
size_t LitHeadAggregate::hash() const {
    return get_value_hash(typeid(LitHeadAggregate).hash_code(), size_t(fun), bounds, elems);
}
size_t Disjunction::hash() const { 
    return get_value_hash(typeid(Disjunction).hash_code(), elems);
}
size_t SimpleHeadLiteral::hash() const { 
    return get_value_hash(typeid(SimpleHeadLiteral).hash_code(), lit);
}
size_t DisjointAggregate::hash() const { 
    return get_value_hash(typeid(DisjointAggregate).hash_code(), elems);
}

// }}}
// {{{ definition of Aggregate::operator==

bool TupleBodyAggregate::operator==(BodyAggregate const &x) const {
    auto t = dynamic_cast<TupleBodyAggregate const *>(&x);
    return t && naf == t->naf && fun == t->fun && is_value_equal_to(bounds, t->bounds) && is_value_equal_to(elems, t->elems);
}
bool LitBodyAggregate::operator==(BodyAggregate const &x) const {
    auto t = dynamic_cast<LitBodyAggregate const *>(&x);
    return t && naf == t->naf && fun == t->fun && is_value_equal_to(bounds, t->bounds) && is_value_equal_to(elems, t->elems);
}
bool Conjunction::operator==(BodyAggregate const &x) const { 
    auto t = dynamic_cast<Conjunction const *>(&x);
    return t && is_value_equal_to(head, t->head) && is_value_equal_to(cond, t->cond);
}
bool SimpleBodyLiteral::operator==(BodyAggregate const &x) const { 
    auto t = dynamic_cast<SimpleBodyLiteral const *>(&x);
    return t && is_value_equal_to(lit, t->lit);
}
bool TupleHeadAggregate::operator==(HeadAggregate const &x) const {
    auto t = dynamic_cast<TupleHeadAggregate const *>(&x);
    return t && fun == t->fun && is_value_equal_to(bounds, t->bounds) && is_value_equal_to(elems, t->elems);
}
bool LitHeadAggregate::operator==(HeadAggregate const &x) const {
    auto t = dynamic_cast<LitHeadAggregate const *>(&x);
    return t && fun == t->fun && is_value_equal_to(bounds, t->bounds) && is_value_equal_to(elems, t->elems);
}
bool Disjunction::operator==(HeadAggregate const &x) const { 
    auto t = dynamic_cast<Disjunction const *>(&x);
    return t && is_value_equal_to(elems, t->elems);
}
bool SimpleHeadLiteral::operator==(HeadAggregate const &x) const { 
    auto t = dynamic_cast<SimpleHeadLiteral const *>(&x);
    return t && is_value_equal_to(lit, t->lit);
}
bool DisjointAggregate::operator==(BodyAggregate const &x) const { 
    auto t = dynamic_cast<DisjointAggregate const *>(&x);
    return t && is_value_equal_to(elems, t->elems);
}

// }}}
// {{{ definition of Aggregate::clone

TupleBodyAggregate *TupleBodyAggregate::clone() const {
    return make_locatable<TupleBodyAggregate>(loc(), naf, fun, get_clone(bounds), get_clone(elems)).release();
}
LitBodyAggregate *LitBodyAggregate::clone() const {
    return make_locatable<LitBodyAggregate>(loc(), naf, fun, get_clone(bounds), get_clone(elems)).release();
}
Conjunction *Conjunction::clone() const { 
    return make_locatable<Conjunction>(loc(), get_clone(head), get_clone(cond)).release();
}
SimpleBodyLiteral *SimpleBodyLiteral::clone() const { 
    return make_unique<SimpleBodyLiteral>(get_clone(lit)).release();
}
TupleHeadAggregate *TupleHeadAggregate::clone() const {
    return make_locatable<TupleHeadAggregate>(loc(), fun, get_clone(bounds), get_clone(elems)).release();
}
LitHeadAggregate *LitHeadAggregate::clone() const {
    return make_locatable<LitHeadAggregate>(loc(), fun, get_clone(bounds), get_clone(elems)).release();
}
Disjunction *Disjunction::clone() const { 
    return make_locatable<Disjunction>(loc(), get_clone(elems)).release();
}
SimpleHeadLiteral *SimpleHeadLiteral::clone() const { 
    return make_unique<SimpleHeadLiteral>(get_clone(lit)).release();
}
DisjointAggregate *DisjointAggregate::clone() const { 
    return make_locatable<DisjointAggregate>(loc(), naf, get_clone(elems)).release();
}

// }}}
// {{{ definition of Aggregate::unpool

namespace {

std::function<ULitVec(ULit const &)> _unpool_lit(bool beforeRewrite) {
    return [beforeRewrite](ULit const &x) { return x->unpool(beforeRewrite); };
}
auto _unpool_bound = [](Bound &x) { return x.unpool(); };

} // namespace

void TupleBodyAggregate::unpool(UBodyAggrVec &x, bool beforeRewrite) {
    BodyAggrElemVec e;
    for (auto &elem : elems) {
        Term::unpoolJoin(elem.second, _unpool_lit(beforeRewrite));
        auto f = [&](UTermVec &&y) { e.emplace_back(std::move(y), get_clone(elem.second)); };
        Term::unpool(elem.first.begin(), elem.first.end(), Gringo::unpool, f);
    }
    auto f = [&](BoundVec &&y) { x.emplace_back(make_locatable<TupleBodyAggregate>(loc(), naf, fun, std::move(y), get_clone(e))); };
    Term::unpool(bounds.begin(), bounds.end(), _unpool_bound, f);
}
void LitBodyAggregate::unpool(UBodyAggrVec &x, bool beforeRewrite) {
    CondLitVec e;
    for (auto &elem : elems) {
        Term::unpoolJoin(elem.second, _unpool_lit(beforeRewrite));
        auto f = [&](ULit &&y) { e.emplace_back(std::move(y), get_clone(elem.second)); };
        Term::unpool(elem.first, _unpool_lit(beforeRewrite), f);
    }
    auto f = [&](BoundVec &&y) { x.emplace_back(make_locatable<LitBodyAggregate>(loc(), naf, fun, std::move(y), get_clone(e))); };
    Term::unpool(bounds.begin(), bounds.end(), _unpool_bound, f);
}
void Conjunction::unpool(UBodyAggrVec &x, bool beforeRewrite) { 
    Term::unpoolJoin(cond, _unpool_lit(beforeRewrite));
    auto f = [&](ULit &&y) { x.emplace_back(make_locatable<Conjunction>(loc(), std::move(y), get_clone(cond))); };
    Term::unpool(head, _unpool_lit(beforeRewrite), f);
}
void SimpleBodyLiteral::unpool(UBodyAggrVec &x, bool beforeRewrite) { 
    for (auto &y : lit->unpool(beforeRewrite)) { x.emplace_back(make_unique<SimpleBodyLiteral>(std::move(y))); }
}
void TupleHeadAggregate::unpool(UHeadAggrVec &x, bool beforeRewrite) {
    HeadAggrElemVec e;
    for (auto &elem : elems) {
        Term::unpoolJoin(std::get<2>(elem), _unpool_lit(beforeRewrite));
        auto f = [&](UTermVec &&y) { e.emplace_back(std::move(y), get_clone(std::get<1>(elem)), get_clone(std::get<2>(elem))); };
        Term::unpool(std::get<0>(elem).begin(), std::get<0>(elem).end(), Gringo::unpool, f);
    }
    elems.clear();
    for (auto &elem : e) {
        auto f = [&](ULit &&y) { elems.emplace_back(get_clone(std::get<0>(elem)), std::move(y), get_clone(std::get<2>(elem))); };
        Term::unpool(std::get<1>(elem), _unpool_lit(beforeRewrite), f);
    }
    auto f = [&](BoundVec &&y) { x.emplace_back(make_locatable<TupleHeadAggregate>(loc(), fun, std::move(y), get_clone(elems))); };
    Term::unpool(bounds.begin(), bounds.end(), _unpool_bound, f);
}
void LitHeadAggregate::unpool(UHeadAggrVec &x, bool beforeRewrite) {
    CondLitVec e;
    for (auto &elem : elems) {
        Term::unpoolJoin(elem.second, _unpool_lit(beforeRewrite));
        auto f = [&](ULit &&y) { e.emplace_back(std::move(y), get_clone(elem.second)); };
        Term::unpool(elem.first, _unpool_lit(beforeRewrite), f);
    }
    auto f = [&](BoundVec &&y) { x.emplace_back(make_locatable<LitHeadAggregate>(loc(), fun, std::move(y), get_clone(e))); };
    Term::unpool(bounds.begin(), bounds.end(), _unpool_bound, f);
}
void Disjunction::unpool(UHeadAggrVec &x, bool beforeRewrite) {
    for (auto &elem : elems) {
        Term::unpoolJoin(elem.second, _unpool_lit(beforeRewrite));
    }
    auto unpool_elem = [&](CondLitVec::value_type const &y) -> CondLitVec {
        CondLitVec ret;
        for (auto &z : y.first->unpool(beforeRewrite)) {
            ret.emplace_back(std::move(z), get_clone(y.second));
        }
        return ret;
    };
    auto f = [&](CondLitVec &&y) { x.emplace_back(make_locatable<Disjunction>(loc(), std::move(y))); };
    Term::unpool(elems.begin(), elems.end(), unpool_elem, f);
}
void SimpleHeadLiteral::unpool(UHeadAggrVec &x, bool beforeRewrite) { 
    for (auto &y : lit->unpool(beforeRewrite)) { x.emplace_back(make_unique<SimpleHeadLiteral>(std::move(y))); }
}
void DisjointAggregate::unpool(UBodyAggrVec &x, bool beforeRewrite) { 
    CSPElemVec e;
    for (auto &elem : elems) {
        Term::unpoolJoin(elem.cond, _unpool_lit(beforeRewrite));
        auto f = [&](UTermVec &&y) { e.emplace_back(elem.loc, std::move(y), get_clone(elem.value), get_clone(elem.cond)); };
        Term::unpool(elem.tuple.begin(), elem.tuple.end(), Gringo::unpool, f);
    }
    elems.clear();
    for (auto &elem : e) {
        for (auto &y : elem.value.unpool()) {
            elems.emplace_back(elem.loc, get_clone(elem.tuple), std::move(y), get_clone(elem.cond));
        }
    }
    x.emplace_back(make_locatable<DisjointAggregate>(loc(), naf, std::move(elems)));
}

// }}}
// {{{ definition of Aggregate::collect

void TupleBodyAggregate::collect(VarTermBoundVec &vars) const {
    for (auto &bound : bounds) { bound.bound->collect(vars, bound.rel == Relation::ASSIGN && naf == NAF::POS); }
    for (auto &elem : elems) {
        for (auto &term : std::get<0>(elem)) { term->collect(vars, false); }
        for (auto &lit : std::get<1>(elem)) { lit->collect(vars, false); }
    }
}
void LitBodyAggregate::collect(VarTermBoundVec &vars) const {
    for (auto &bound : bounds) { bound.bound->collect(vars, bound.rel == Relation::ASSIGN && naf == NAF::POS); }
    for (auto &elem : elems) {
        elem.first->collect(vars, false);
        for (auto &lit : elem.second) { lit->collect(vars, false); }
    }
}
void Conjunction::collect(VarTermBoundVec &vars) const {
    head->collect(vars, false);
    for (auto &lit : cond) { lit->collect(vars, false); }
}
void SimpleBodyLiteral::collect(VarTermBoundVec &vars) const { lit->collect(vars, true); }

void TupleHeadAggregate::collect(VarTermBoundVec &vars) const {
    for (auto &bound : bounds) { bound.bound->collect(vars, false); }
    for (auto &elem : elems) {
        for (auto &term : std::get<0>(elem)) { term->collect(vars, false); }
        std::get<1>(elem)->collect(vars, false);
        for (auto &lit : std::get<2>(elem)) { lit->collect(vars, false); }
    }
}
void LitHeadAggregate::collect(VarTermBoundVec &vars) const {
    for (auto &bound : bounds) { bound.bound->collect(vars, false); }
    for (auto &elem : elems) {
        elem.first->collect(vars, false);
        for (auto &lit : elem.second) { lit->collect(vars, false); }
    }
}
void Disjunction::collect(VarTermBoundVec &vars) const {
    for (auto &elem : elems) {
        elem.first->collect(vars, false);
        for (auto &lit : elem.second) { lit->collect(vars, false); }
    }
}
void SimpleHeadLiteral::collect(VarTermBoundVec &vars) const { lit->collect(vars, true); }
void DisjointAggregate::collect(VarTermBoundVec &vars) const {
    for (auto &elem : elems) {
        for (auto &term : elem.tuple) { term->collect(vars, false); }
        elem.value.collect(vars);
        for (auto &lit : elem.cond) { lit->collect(vars, false); }
    }
}

// }}}
// {{{ definition of Aggregate::rewriteAggregate

bool TupleBodyAggregate::rewriteAggregates(UBodyAggrVec &aggr) {
    BoundVec assign;
    auto jt(bounds.begin());
    for (auto it = jt, ie = bounds.end(); it != ie; ++it) {
        if (it->rel == Relation::ASSIGN && naf == NAF::POS) { assign.emplace_back(std::move(*it)); }
        else {
            if (it != jt) { *jt = std::move(*it); }
            ++jt;
        }
    }
    bounds.erase(jt, bounds.end());
    bool skip = bounds.empty() && !assign.empty();
    for (auto it = assign.begin(), ie = assign.end() - skip; it != ie; ++it) {
        BoundVec bound;
        bound.emplace_back(std::move(*it));
        aggr.emplace_back(make_locatable<TupleBodyAggregate>(loc(), naf, fun, std::move(bound), get_clone(elems)));
    }
    if (skip) { bounds.emplace_back(std::move(assign.back())); }
    return !bounds.empty();
}
bool LitBodyAggregate::rewriteAggregates(UBodyAggrVec &aggr) {
    int id = 0;
    BodyAggrElemVec elems;
    for (auto &x : this->elems) {
        UTermVec tuple;
        tuple.emplace_back(make_locatable<ValTerm>(x.first->loc(), Value(1)));
        x.first->toTuple(tuple, id);
        ULitVec lits(std::move(x.second));
        lits.emplace_back(std::move(x.first));
        elems.emplace_back(std::move(tuple), std::move(lits));
    }
    UBodyAggr x(make_locatable<TupleBodyAggregate>(loc(), naf, fun, std::move(bounds), std::move(elems)));
    if (x->rewriteAggregates(aggr)) { aggr.emplace_back(std::move(x)); }
    return false;
}
bool Conjunction::rewriteAggregates(UBodyAggrVec &) { 
    if (ULit shifted = head->shift(true)) {
        head = make_locatable<FalseLiteral>(head->loc());
        cond.emplace_back(std::move(shifted));
    }
    return true;
}
bool SimpleBodyLiteral::rewriteAggregates(UBodyAggrVec &) { return true; }
bool DisjointAggregate::rewriteAggregates(UBodyAggrVec &) { return true; }
UHeadAggr TupleHeadAggregate::rewriteAggregates(UBodyAggrVec &) { 
    return nullptr;
}
UHeadAggr LitHeadAggregate::rewriteAggregates(UBodyAggrVec &) {
    throw std::runtime_error("LitHeadAggregate::rewriteAggregates: must not be called");
}
UHeadAggr Disjunction::rewriteAggregates(UBodyAggrVec &) { 
    return nullptr;
}
UHeadAggr SimpleHeadLiteral::rewriteAggregates(UBodyAggrVec &aggr) { 
    ULit shifted(lit->shift(true));
    if (shifted) { 
        aggr.emplace_back(make_unique<SimpleBodyLiteral>(std::move(shifted)));
        return make_unique<SimpleHeadLiteral>(make_locatable<FalseLiteral>(lit->loc()));
    }
    return nullptr;
}

// }}}
// {{{ definition of Aggregate::rewriteAggregate

UHeadAggr TupleHeadAggregate::shiftHead(UBodyAggrVec &) { 
    for (auto &x : elems) {
        if (ULit shifted = std::get<1>(x)->shift(false)) {
            std::get<1>(x) = make_locatable<FalseLiteral>(std::get<1>(x)->loc());
            std::get<2>(x).emplace_back(std::move(shifted));
        }
    }
    return nullptr;
}
UHeadAggr LitHeadAggregate::shiftHead(UBodyAggrVec &aggr) {
    int id = 0;
    HeadAggrElemVec elems;
    for (auto &x : this->elems) {
        UTermVec tuple;
        tuple.emplace_back(make_locatable<ValTerm>(x.first->loc(), Value(1)));
        x.first->toTuple(tuple, id);
        elems.emplace_back(std::move(tuple), std::move(x.first), std::move(x.second));
    }
    UHeadAggr x(make_locatable<TupleHeadAggregate>(loc(), fun, std::move(bounds), std::move(elems)));
    Term::replace(x, x->shiftHead(aggr));
    return x;
}
UHeadAggr Disjunction::shiftHead(UBodyAggrVec &) { 
    for (auto &x : elems) {
        if (ULit shifted = x.first->shift(false)) {
            x.first = make_locatable<FalseLiteral>(x.first->loc());
            x.second.emplace_back(std::move(shifted));
        }
    }
    return nullptr;
}
UHeadAggr SimpleHeadLiteral::shiftHead(UBodyAggrVec &) { 
    return nullptr;
}

// }}}
// {{{ definition of Aggregate::simplify

void TupleBodyAggregate::simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum) {
    for (auto &bound : bounds) { bound.simplify(dots, scripts, auxNum); }
    for (auto &elem : elems) {
        Term::DotsMap dotsElem;
        Term::ScriptMap scriptsElem;
        for (auto &term : std::get<0>(elem)) { term->simplify(dotsElem, scriptsElem, auxNum, false, false).update(term); }
        for (auto &lit : std::get<1>(elem)) { lit->simplify(project, dotsElem, scriptsElem, auxNum); }
        for (auto &dot : dotsElem) { std::get<1>(elem).emplace_back(RangeLiteral::make(dot)); }
        for (auto &script : scriptsElem) { std::get<1>(elem).emplace_back(ScriptLiteral::make(script)); }
    }
}
void LitBodyAggregate::simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum) {
    for (auto &bound : bounds) { bound.simplify(dots, scripts, auxNum); }
    for (auto &elem : elems) {
        Term::DotsMap dotsElem;
        Term::ScriptMap scriptsElem;
        std::get<0>(elem)->simplify(project, dotsElem, scriptsElem, auxNum);
        for (auto &lit : std::get<1>(elem)) { lit->simplify(project, dotsElem, scriptsElem, auxNum); }
        for (auto &dot : dotsElem) { std::get<1>(elem).emplace_back(RangeLiteral::make(dot)); }
        for (auto &script : scriptsElem) { std::get<1>(elem).emplace_back(ScriptLiteral::make(script)); }
    }
}
void Conjunction::simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum) { 
    head->simplify(project, dots, scripts, auxNum);
    Term::DotsMap dotsElem;
    Term::ScriptMap scriptsElem;
    for (auto &lit : cond) { lit->simplify(project, dotsElem, scriptsElem, auxNum); }
    for (auto &dot : dotsElem) { cond.emplace_back(RangeLiteral::make(dot)); }
    for (auto &script : scriptsElem) { cond.emplace_back(ScriptLiteral::make(script)); }
}
void SimpleBodyLiteral::simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum) {
    lit->simplify(project, dots, scripts, auxNum);
}
void TupleHeadAggregate::simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum) {
    for (auto &bound : bounds) { bound.simplify(dots, scripts, auxNum); }
    for (auto &elem : elems) {
        Term::DotsMap dotsElem;
        Term::ScriptMap scriptsElem;
        for (auto &term : std::get<0>(elem)) { term->simplify(dotsElem, scriptsElem, auxNum, false, false).update(term); }
        std::get<1>(elem)->simplify(project, dotsElem, scriptsElem, auxNum, false);
        for (auto &lit : std::get<2>(elem)) { lit->simplify(project, dotsElem, scriptsElem, auxNum); }
        for (auto &dot : dotsElem) { std::get<2>(elem).emplace_back(RangeLiteral::make(dot)); }
        for (auto &script : scriptsElem) { std::get<2>(elem).emplace_back(ScriptLiteral::make(script)); }
    }
}
void LitHeadAggregate::simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum) {
    for (auto &bound : bounds) { bound.simplify(dots, scripts, auxNum); }
    for (auto &elem : elems) {
        Term::DotsMap dotsElem;
        Term::ScriptMap scriptsElem;
        std::get<0>(elem)->simplify(project, dotsElem, scriptsElem, auxNum, false);
        for (auto &lit : std::get<1>(elem)) { lit->simplify(project, dotsElem, scriptsElem, auxNum); }
        for (auto &dot : dotsElem) { std::get<1>(elem).emplace_back(RangeLiteral::make(dot)); }
        for (auto &script : scriptsElem) { std::get<1>(elem).emplace_back(ScriptLiteral::make(script)); }
    }
}
void Disjunction::simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum) { 
    for (auto &elem : elems) {
        elem.first->simplify(project, dots, scripts, auxNum, false);
        Term::DotsMap dotsElem;
        Term::ScriptMap scriptsElem;
        for (auto &lit : elem.second) { lit->simplify(project, dotsElem, scriptsElem, auxNum); }
        for (auto &dot : dotsElem) { elem.second.emplace_back(RangeLiteral::make(dot)); }
        for (auto &script : scriptsElem) { std::get<1>(elem).emplace_back(ScriptLiteral::make(script)); }
    }
}
void SimpleHeadLiteral::simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum) {
    lit->simplify(project, dots, scripts, auxNum, false);
}
void DisjointAggregate::simplify(Projections &project, Term::DotsMap &, Term::ScriptMap &, unsigned &auxNum) {
    for (auto &elem : elems) {
        Term::DotsMap dotsElem;
        Term::ScriptMap scriptsElem;
        for (auto &term : elem.tuple) { term->simplify(dotsElem, scriptsElem, auxNum, false, false).update(term); }
        elem.value.simplify(dotsElem, scriptsElem, auxNum);
        for (auto &lit : elem.cond) { lit->simplify(project, dotsElem, scriptsElem, auxNum); }
        for (auto &dot : dotsElem) { elem.cond.emplace_back(RangeLiteral::make(dot)); }
        for (auto &script : scriptsElem) { elem.cond.emplace_back(ScriptLiteral::make(script)); }
    }
}
// }}}
// {{{ definition of Aggregate::rewriteArithmetics

void TupleBodyAggregate::rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &, unsigned &auxNum) {
    for (auto &bound : bounds) { bound.rewriteArithmetics(arith, auxNum); }
    for (auto &elem : elems) {
        Literal::AssignVec assign;
        arith.emplace_back();
        for (auto &y : std::get<1>(elem)) { y->rewriteArithmetics(arith, assign, auxNum); }
        for (auto &y : arith.back()) { std::get<1>(elem).emplace_back(RelationLiteral::make(y)); }
        for (auto &y : assign) { std::get<1>(elem).emplace_back(RelationLiteral::make(y)); }
        arith.pop_back();
    }
}
void LitBodyAggregate::rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &, unsigned &auxNum) {
    for (auto &bound : bounds) { bound.rewriteArithmetics(arith, auxNum); }
    for (auto &elem : elems) {
        Literal::AssignVec assign;
        arith.emplace_back();
        for (auto &y : std::get<1>(elem)) { y->rewriteArithmetics(arith, assign, auxNum); }
        for (auto &y : arith.back()) { std::get<1>(elem).emplace_back(RelationLiteral::make(y)); }
        for (auto &y : assign) { std::get<1>(elem).emplace_back(RelationLiteral::make(y)); }
        arith.pop_back();
    }
}
void Conjunction::rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &, unsigned &auxNum) { 
    Literal::AssignVec assign;
    arith.emplace_back();
    for (auto &y : cond) { y->rewriteArithmetics(arith, assign, auxNum); }
    for (auto &y : arith.back()) { cond.emplace_back(RelationLiteral::make(y)); }
    for (auto &y : assign) { cond.emplace_back(RelationLiteral::make(y)); }
    arith.pop_back();
}
void SimpleBodyLiteral::rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &assign, unsigned &auxNum) {
    lit->rewriteArithmetics(arith, assign, auxNum);
}
void TupleHeadAggregate::rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum) {
    for (auto &bound : bounds) { bound.rewriteArithmetics(arith, auxNum); }
    for (auto &elem : elems) {
        Literal::AssignVec assign;
        arith.emplace_back();
        for (auto &y : std::get<2>(elem)) { y->rewriteArithmetics(arith, assign, auxNum); }
        for (auto &y : arith.back()) { std::get<2>(elem).emplace_back(RelationLiteral::make(y)); }
        for (auto &y : assign) { std::get<2>(elem).emplace_back(RelationLiteral::make(y)); }
        arith.pop_back();
    }
}
void LitHeadAggregate::rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum) {
    for (auto &bound : bounds) { bound.rewriteArithmetics(arith, auxNum); }
    for (auto &elem : elems) {
        Literal::AssignVec assign;
        arith.emplace_back();
        for (auto &y : std::get<1>(elem)) { y->rewriteArithmetics(arith, assign, auxNum); }
        for (auto &y : arith.back()) { std::get<1>(elem).emplace_back(RelationLiteral::make(y)); }
        for (auto &y : assign) { std::get<1>(elem).emplace_back(RelationLiteral::make(y)); }
        arith.pop_back();
    }
}
void Disjunction::rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum) { 
    for (auto &elem : elems) {
        Literal::AssignVec assign;
        arith.emplace_back();
        for (auto &y : std::get<1>(elem)) { y->rewriteArithmetics(arith, assign, auxNum); }
        for (auto &y : arith.back()) { std::get<1>(elem).emplace_back(RelationLiteral::make(y)); }
        for (auto &y : assign) { std::get<1>(elem).emplace_back(RelationLiteral::make(y)); }
        arith.pop_back();
    }
}
void SimpleHeadLiteral::rewriteArithmetics(Term::ArithmeticsMap &, unsigned &) { 
    // Note: nothing to do
}
void DisjointAggregate::rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &, unsigned &auxNum) {
    for (auto &elem : elems) {
        Literal::AssignVec assign;
        arith.emplace_back();
        for (auto &y : elem.cond) { y->rewriteArithmetics(arith, assign, auxNum); }
        for (auto &y : arith.back()) { elem.cond.emplace_back(RelationLiteral::make(y)); }
        for (auto &y : assign) { elem.cond.emplace_back(RelationLiteral::make(y)); }
        arith.pop_back();
    }
}
// }}}
// {{{ definition of Aggregate::assignLevels

void TupleBodyAggregate::assignLevels(AssignLevel &lvl) {
    VarTermBoundVec vars;
    for (auto &bound : bounds) { bound.bound->collect(vars, false); }
    lvl.add(vars);
    for (auto &elem : elems) {
        AssignLevel &local(lvl.subLevel());
        VarTermBoundVec vars;
        for (auto &term : std::get<0>(elem)) { term->collect(vars, false); }
        for (auto &lit : std::get<1>(elem)) { lit->collect(vars, false); }
        local.add(vars);
    }
}
void LitBodyAggregate::assignLevels(AssignLevel &lvl) {
    VarTermBoundVec vars;
    for (auto &bound : bounds) { bound.bound->collect(vars, false); }
    lvl.add(vars);
    for (auto &elem : elems) {
        AssignLevel &local(lvl.subLevel());
        VarTermBoundVec vars;
        std::get<0>(elem)->collect(vars, false);
        for (auto &lit : std::get<1>(elem)) { lit->collect(vars, false); }
        local.add(vars);
    }
}
void Conjunction::assignLevels(AssignLevel &lvl) {
    AssignLevel &local(lvl.subLevel());
    VarTermBoundVec vars;
    head->collect(vars, false);
    for (auto &lit : cond) { lit->collect(vars, false); }
    local.add(vars);
}
void SimpleBodyLiteral::assignLevels(AssignLevel &lvl) { 
    VarTermBoundVec vars;
    lit->collect(vars, false);
    lvl.add(vars);
}
void TupleHeadAggregate::assignLevels(AssignLevel &lvl) {
    VarTermBoundVec vars;
    for (auto &bound : bounds) { bound.bound->collect(vars, false); }
    lvl.add(vars);
    for (auto &elem : elems) {
        AssignLevel &local(lvl.subLevel());
        VarTermBoundVec vars;
        for (auto &term : std::get<0>(elem)) { term->collect(vars, false); }
        std::get<1>(elem)->collect(vars, false);
        for (auto &lit : std::get<2>(elem)) { lit->collect(vars, false); }
        local.add(vars);
    }
}
void LitHeadAggregate::assignLevels(AssignLevel &lvl) {
    VarTermBoundVec vars;
    for (auto &bound : bounds) { bound.bound->collect(vars, false); }
    lvl.add(vars);
    for (auto &elem : elems) {
        AssignLevel &local(lvl.subLevel());
        VarTermBoundVec vars;
        std::get<0>(elem)->collect(vars, false);
        for (auto &lit : std::get<1>(elem)) { lit->collect(vars, false); }
        local.add(vars);
    }
}
void Disjunction::assignLevels(AssignLevel &lvl) {
    for (auto &elem : elems) {
        AssignLevel &local(lvl.subLevel());
        VarTermBoundVec vars;
        std::get<0>(elem)->collect(vars, false);
        for (auto &lit : std::get<1>(elem)) { lit->collect(vars, false); }
        local.add(vars);
    }
}
void SimpleHeadLiteral::assignLevels(AssignLevel &lvl) {
    VarTermBoundVec vars;
    lit->collect(vars, false);
    lvl.add(vars);
}
void DisjointAggregate::assignLevels(AssignLevel &lvl) {
    for (auto &elem : elems) {
        AssignLevel &local(lvl.subLevel());
        VarTermBoundVec vars;
        for (auto &term : elem.tuple) { term->collect(vars, false); }
        elem.value.collect(vars);
        for (auto &lit : elem.cond) { lit->collect(vars, false); }
        local.add(vars);
    }
}

// }}}
// {{{ definition of Aggregate::isEDB

Value HeadAggregate::isEDB() const     { return Value(); }
Value SimpleHeadLiteral::isEDB() const { return lit->isEDB(); }

// }}}
// {{{ definition of Aggregate::checkLevels

namespace {

// TODO: consider adding a more intelligent parameter to check that does all of this
void _add(ChkLvlVec &levels, VarTermBoundVec &vars) {
    for (auto &x: vars) { 
        auto &lvl(levels[x.first->level]);
        bool bind = x.second && levels.size() == x.first->level + 1;
        if (bind) { lvl.dep.insertEdge(*lvl.current, lvl.var(*x.first)); }
        else      { lvl.dep.insertEdge(lvl.var(*x.first), *lvl.current); }
    }
}

void _add(ChkLvlVec &levels, ULit const &lit, bool bind) {
    VarTermBoundVec vars;
    levels.back().current = &levels.back().dep.insertEnt();
    lit->collect(vars, bind);
    _add(levels, vars);
}

void _add(ChkLvlVec &levels, UTermVec const &terms, CSPAddTerm const *term = nullptr) {
    VarTermBoundVec vars;
    levels.back().current = &levels.back().dep.insertEnt();
    for (auto &x : terms)  { x->collect(vars, false); }
    if (term) { term->collect(vars); }
    _add(levels, vars);
}


void _add(ChkLvlVec &levels, ULitVec const &cond) {
    for (auto &x : cond) { _add(levels, x, true); }
}

template <class T>
bool _aggr(ChkLvlVec &levels, BoundVec const &bounds, T const &f, bool bind) {
    bool ret = true;
    bool assign = false;
    CheckLevel::SC::EntNode *depend = 0;
    for (auto &y : bounds) {
        if (bind && y.rel == Relation::ASSIGN) {
            levels.back().current = &levels.back().dep.insertEnt();
            VarTermBoundVec vars;
            y.bound->collect(vars, true);
            _add(levels, vars);
            ret = f() && ret;
            assign = true;
        }
        else { 
            if (!depend) { depend = &levels.back().dep.insertEnt(); }
            levels.back().current = depend;
            VarTermBoundVec vars;
            y.bound->collect(vars, false);
            _add(levels, vars);
        }
    }
    if (!depend && !assign) { depend = &levels.back().dep.insertEnt(); }
    if (depend) {
        levels.back().current = depend;
        ret = f() && ret;
    }
    return ret;
}

} // namespace

bool TupleBodyAggregate::check(ChkLvlVec &levels) const {
    auto f = [&]() -> bool {
        bool ret = true;
        for (auto &y : elems) {
            levels.emplace_back(loc(), *this);
            _add(levels, y.first);
            _add(levels, y.second);
            ret = levels.back().check() && ret;
            levels.pop_back();
        }
        return ret;
    };
    return _aggr(levels, bounds, f, naf == NAF::POS);
}
bool LitBodyAggregate::check(ChkLvlVec &levels) const {
    auto f = [&]() -> bool {
        bool ret = true;
        for (auto &y : elems) {
            levels.emplace_back(loc(), *this);
            _add(levels, y.first, false);
            _add(levels, y.second);
            ret = levels.back().check() && ret;
            levels.pop_back();
        }
        return ret;
    };
    return _aggr(levels, bounds, f, naf == NAF::POS);
}
bool Conjunction::check(ChkLvlVec &levels) const {
    levels.back().current = &levels.back().dep.insertEnt();
    levels.emplace_back(loc(), *this);
    _add(levels, head, false);
    _add(levels, cond);
    bool ret = levels.back().check();
    levels.pop_back();
    return ret;
}
bool SimpleBodyLiteral::check(ChkLvlVec &levels) const { 
    levels.back().current = &levels.back().dep.insertEnt();
    _add(levels, lit, true);
    return true;
}
bool TupleHeadAggregate::check(ChkLvlVec &levels) const {
    auto f = [&]() -> bool {
        bool ret = true;
        for (auto &y : elems) {
            levels.emplace_back(loc(), *this);
            _add(levels, std::get<0>(y));
            _add(levels, std::get<1>(y), false);
            _add(levels, std::get<2>(y));
            ret = levels.back().check() && ret;
            levels.pop_back();
        }
        return ret;
    };
    return _aggr(levels, bounds, f, false);
}
bool LitHeadAggregate::check(ChkLvlVec &levels) const {
    auto f = [&]() -> bool {
        bool ret = true;
        for (auto &y : elems) {
            levels.emplace_back(loc(), *this);
            _add(levels, y.first, false);
            _add(levels, y.second);
            ret = levels.back().check() && ret;
            levels.pop_back();
        }
        return ret;
    };
    return _aggr(levels, bounds, f, false);
}
bool Disjunction::check(ChkLvlVec &levels) const {
    bool ret = true;
    levels.back().current = &levels.back().dep.insertEnt();
    for (auto &y : elems) {
        levels.emplace_back(loc(), *this);
        _add(levels, y.first, false);
        _add(levels, y.second);
        ret = levels.back().check() && ret;
        levels.pop_back();
    }
    return ret;
}
bool SimpleHeadLiteral::check(ChkLvlVec &levels) const {
    levels.back().current = &levels.back().dep.insertEnt();
    _add(levels, lit, false);
    return true;
}
bool DisjointAggregate::check(ChkLvlVec &levels) const {
    auto f = [&]() -> bool {
        bool ret = true;
        for (auto &y : elems) {
            levels.emplace_back(loc(), *this);
            _add(levels, y.tuple, &y.value);
            _add(levels, y.cond);
            ret = levels.back().check() && ret;
            levels.pop_back();
        }
        return ret;
    };
    return _aggr(levels, {}, f, false);
}

// }}}
// {{{ definition of Aggregate::hasPool

bool TupleBodyAggregate::hasPool(bool beforeRewrite) const {
    for (auto &bound : bounds) { if (bound.bound->hasPool()) { return true; } }
    for (auto &elem : elems) {
        for (auto &term : elem.first) { if (term->hasPool()) { return true; } }
        for (auto &lit : elem.second) { if (lit->hasPool(beforeRewrite)) { return true; } }
    }
    return false;
}
bool LitBodyAggregate::hasPool(bool beforeRewrite) const {
    for (auto &bound : bounds) { if (bound.bound->hasPool()) { return true; } }
    for (auto &elem : elems) {
        if (elem.first->hasPool(beforeRewrite)) { return true; }
        for (auto &lit : elem.second) { if (lit->hasPool(beforeRewrite)) { return true; } }
    }
    return false;
}
bool Conjunction::hasPool(bool beforeRewrite) const { 
    if (head->hasPool(beforeRewrite)) { return true; }
    for (auto &lit : cond) { if (lit->hasPool(beforeRewrite)) { return true; } }
    return false;
}
bool SimpleBodyLiteral::hasPool(bool beforeRewrite) const { 
    return lit->hasPool(beforeRewrite);
}
bool TupleHeadAggregate::hasPool(bool beforeRewrite) const {
    for (auto &bound : bounds) { if (bound.bound->hasPool()) { return true; } }
    for (auto &elem : elems) {
        for (auto &term : std::get<0>(elem)) { if (term->hasPool()) { return true; } }
        if (std::get<1>(elem)->hasPool(beforeRewrite)) { return true; }
        for (auto &lit : std::get<2>(elem)) { if (lit->hasPool(beforeRewrite)) { return true; } }
    }
    return false;
}
bool LitHeadAggregate::hasPool(bool beforeRewrite) const {
    for (auto &bound : bounds) { if (bound.bound->hasPool()) { return true; } }
    for (auto &elem : elems) {
        if (elem.first->hasPool(beforeRewrite)) { return true; }
        for (auto &lit : elem.second) { if (lit->hasPool(beforeRewrite)) { return true; } }
    }
    return false;
}
bool Disjunction::hasPool(bool beforeRewrite) const {
    for (auto &elem : elems) {
        if (elem.first->hasPool(beforeRewrite)) { return true; }
        for (auto &lit : elem.second) { if (lit->hasPool(beforeRewrite)) { return true; } }
    }
    return false;
}
bool SimpleHeadLiteral::hasPool(bool beforeRewrite) const { 
    return lit->hasPool(beforeRewrite);
}
bool DisjointAggregate::hasPool(bool beforeRewrite) const {
    for (auto &elem : elems) {
        for (auto &term : elem.tuple) { if (term->hasPool()) { return true; } }
        if (elem.value.hasPool()) { return true; }
        for (auto &lit : elem.cond) { if (lit->hasPool(beforeRewrite)) { return true; } }
    }
    return false;
}

// }}}
// {{{ definition of Aggregate::replace

void TupleBodyAggregate::replace(Defines &x) {
    for (auto &bound : bounds) { Term::replace(bound.bound, bound.bound->replace(x, true)); }
    for (auto &elem : elems) {
        for (auto &y : elem.first)  { Term::replace(y, y->replace(x, true)); }
        for (auto &y : elem.second) { y->replace(x); }
    }
}
void LitBodyAggregate::replace(Defines &x) {
    for (auto &bound : bounds) { Term::replace(bound.bound, bound.bound->replace(x, true)); }
    for (auto &elem : elems) {
        elem.first->replace(x);
        for (auto &y : elem.second) { y->replace(x); }
    }
}
void Conjunction::replace(Defines &x) { 
    head->replace(x);
    for (auto &y : cond) { y->replace(x); }
}
void SimpleBodyLiteral::replace(Defines &x) {
    lit->replace(x);
}
void TupleHeadAggregate::replace(Defines &x) {
    for (auto &bound : bounds) { Term::replace(bound.bound, bound.bound->replace(x, true)); }
    for (auto &elem : elems) {
        for (auto &y : std::get<0>(elem)) { Term::replace(y, y->replace(x, true)); }
        std::get<1>(elem)->replace(x);
        for (auto &y : std::get<2>(elem)) { y->replace(x); }
    }
}
void LitHeadAggregate::replace(Defines &x) {
    for (auto &bound : bounds) { Term::replace(bound.bound, bound.bound->replace(x, true)); }
    for (auto &elem : elems) {
        elem.first->replace(x);
        for (auto &y : elem.second) { y->replace(x); }
    }
}
void Disjunction::replace(Defines &x) { 
    for (auto &elem : elems) {
        elem.first->replace(x);
        for (auto &y : elem.second) { y->replace(x); }
    }
}
void SimpleHeadLiteral::replace(Defines &x) { 
    lit->replace(x);
}
void DisjointAggregate::replace(Defines &x) { 
    for (auto &elem : elems) {
        for (auto &y : elem.tuple) { y->replace(x, true); }
        elem.value.replace(x);
        for (auto &y : elem.cond) { y->replace(x); }
    }
}

// }}}
// {{{ definition of Aggregate::toGround

CreateBody TupleBodyAggregate::toGround(ToGroundArg &x, Ground::UStmVec &) const {
    if (!isAssignment()) {
        Ground::SBodyAggregateDomain dom(std::make_shared<Ground::BodyAggregateDomain>(x.newId(*this), get_clone(bounds), fun));
        CreateStmVec split;
        split.emplace_back([dom,this](Ground::ULitVec &&auxLits) -> Ground::UStm {
            UTermVec tuple;
            tuple.emplace_back(make_locatable<ValTerm>(loc(), Value()));
            // Note: should this become a function?
            UTerm neutral;
            switch (fun) {
                case AggregateFunction::MIN: { neutral = make_locatable<ValTerm>(loc(), Value(false));  break; }
                case AggregateFunction::MAX: { neutral = make_locatable<ValTerm>(loc(), Value(true)); break; }
                default:                     { neutral = make_locatable<ValTerm>(loc(), Value(0));     break; }
            }
            for (auto &y : bounds) { auxLits.emplace_back(make_unique<Ground::RelationLiteral>(y.rel, get_clone(neutral), get_clone(y.bound))); }
            return make_unique<Ground::BodyAggregateAccumulate>(dom, get_clone(tuple), Ground::ULitVec(), std::move(auxLits));
        });
        for (auto &y : elems) { 
            split.emplace_back([this,dom,&y,&x](Ground::ULitVec &&auxLits) -> Ground::UStm {
                Ground::ULitVec lits;
                for (auto &z : y.second) { lits.emplace_back(z->toGround(x.domains)); }
                return make_unique<Ground::BodyAggregateAccumulate>(dom, get_clone(y.first), std::move(lits), std::move(auxLits));
            });
        }
        return CreateBody([dom, this](Ground::ULitVec &lits, bool primary) {
            if (primary) { lits.emplace_back(make_unique<Ground::BodyAggregateLiteral>(dom, naf)); }
        }, std::move(split));
    }
    else {
        assert(bounds.size() == 1);
        VarTermBoundVec vars;
        for (auto &y : elems) {
            for (auto &z : y.first)  { z->collect(vars, false); }
            for (auto &z : y.second) { z->collect(vars, false); }
        }
        UTermVec global(x.getGlobal(vars));
        global.emplace_back(get_clone(bounds.front().bound));
        UTermVec globalSpecial(x.getGlobal(vars));
        UTerm repr(x.newId(std::move(global), loc(), false));
        UTerm specialRepr(x.newId(std::move(globalSpecial), loc()));
        Ground::SAssignmentAggregateDomain dom(std::make_shared<Ground::AssignmentAggregateDomain>(std::move(repr), std::move(specialRepr), fun));
        CreateStmVec split;
        split.emplace_back([dom, this](Ground::ULitVec &&auxLits) -> Ground::UStm {
            UTermVec tuple;
            tuple.emplace_back(make_locatable<ValTerm>(loc(), Value()));
            return make_unique<Ground::AssignmentAggregateAccumulate>(dom, tuple, Ground::ULitVec(), std::move(auxLits));
        });
        for (auto &y : elems) {
            split.emplace_back([this, dom, &y, &x](Ground::ULitVec &&auxLits) -> Ground::UStm {
                Ground::ULitVec lits;
                for (auto &z : y.second) { lits.emplace_back(z->toGround(x.domains)); }
                return make_unique<Ground::AssignmentAggregateAccumulate>(dom, y.first, std::move(lits), std::move(auxLits));
            });
        }
        return CreateBody([dom, &x](Ground::ULitVec &lits, bool primary) {
            if (primary) { lits.emplace_back(make_unique<Ground::AssignmentAggregateLiteral>(dom)); }
        }, std::move(split));
    }
}
CreateBody LitBodyAggregate::toGround(ToGroundArg &, Ground::UStmVec &) const {
    throw std::logic_error("Aggregate::rewriteAggregates must be called before LitAggregate::toGround");
}
CreateBody Conjunction::toGround(ToGroundArg &x, Ground::UStmVec &stms) const {
    UTerm domRepr(x.newId(*this));
    UTerm headRepr(head->headRepr());
    PredicateDomain *predDom{nullptr};
    PredicateDomain *headDom{nullptr};
    UTerm predRepr;
    if (headRepr) {
        VarTermBoundVec vars;
        domRepr->collect(vars, false);
        headRepr->collect(vars, false);
        predRepr = x.newId(x.getGlobal(std::move(vars)), loc());
        predDom = &add(x.domains, predRepr->getSig());
        headDom = &add(x.domains, headRepr->getSig());
    }
    Ground::ULitVec lits;
    for (auto &y : cond) { lits.emplace_back(y->toGround(x.domains)); }
    Ground::SConjunctionDomain dom(std::make_shared<Ground::ConjunctionDomain>(
        std::move(domRepr),
        predDom,
        std::move(predRepr),
        headDom,
        std::move(headRepr),
        std::move(lits)));
    if (predDom) {
        lits.clear();
        lits.emplace_back(make_unique<Ground::PredicateLiteral>(*predDom, NAF::POS, get_clone(dom->head->predRep)));
        lits.emplace_back(head->toGround(x.domains));
        stms.emplace_back(make_unique<Ground::ConjunctionAccumulateFact>(dom, std::move(lits)));
    }
    CreateStmVec split;
    split.emplace_back([dom](Ground::ULitVec &&auxLits) -> Ground::UStm { return make_unique<Ground::ConjunctionAccumulateEmpty>(dom, std::move(auxLits)); });
    split.emplace_back([dom](Ground::ULitVec &&auxLits) -> Ground::UStm { return make_unique<Ground::ConjunctionAccumulate>(dom, std::move(auxLits)); });
    return CreateBody([dom, &x](Ground::ULitVec &lits, bool primary) {
        if (primary) { lits.emplace_back(make_unique<Ground::ConjunctionLiteral>(dom)); }
    }, std::move(split));
}
CreateBody SimpleBodyLiteral::toGround(ToGroundArg &x, Ground::UStmVec &) const {
    return {[&](Ground::ULitVec &lits, bool) -> void {
        lits.emplace_back(lit->toGround(x.domains));
    }, CreateStmVec()};
}
CreateBody DisjointAggregate::toGround(ToGroundArg &x, Ground::UStmVec &) const {
    Ground::SDisjointDomain dom(std::make_shared<Ground::DisjointDomain>(x.newId(*this)));
    CreateStmVec split;
    split.emplace_back([dom,this](Ground::ULitVec &&auxLits) -> Ground::UStm {
        return make_unique<Ground::DisjointAccumulate>(dom, Ground::ULitVec(), std::move(auxLits));
    });
    for (auto &y : elems) { 
        split.emplace_back([this,dom,&y,&x](Ground::ULitVec &&auxLits) -> Ground::UStm {
            Ground::ULitVec lits;
            for (auto &z : y.cond) { lits.emplace_back(z->toGround(x.domains)); }
            return make_unique<Ground::DisjointAccumulate>(dom, get_clone(y.tuple), get_clone(y.value), std::move(lits), std::move(auxLits));
        });
    }
    return CreateBody([dom, this](Ground::ULitVec &lits, bool primary) {
        if (primary) { lits.emplace_back(make_unique<Ground::DisjointLiteral>(dom, naf)); }
    }, std::move(split));
}
CreateHead TupleHeadAggregate::toGround(ToGroundArg &x, bool) const {
    FWString dummy(x.newId());
    Ground::SHeadAggregateDomain dom(std::make_shared<Ground::HeadAggregateDomain>(x.newId(*this), fun, get_clone(bounds), dummy));
    CreateStmVec split;
    for (auto &y : elems) {
        // TODO: after finishing disjunctions consider removing auxlits from the head interface altogether!
        split.emplace_back([this, dom, &y, &x](Ground::ULitVec &&) -> Ground::UStm {
            Ground::ULitVec lits;
            for (auto &z : std::get<2>(y)) { lits.emplace_back(z->toGround(x.domains)); }
            lits.emplace_back(make_unique<Ground::HeadAggregateLiteral>(dom));
            UTerm headRep(std::get<1>(y)->headRepr());
            PredicateDomain *pred = headRep ? &add(x.domains, headRep->getSig()) : nullptr;
            return make_unique<Ground::HeadAggregateAccumulate>(dom, get_clone(std::get<0>(y)), pred, std::move(headRep), std::move(lits));
        });
    }
    return 
        CreateHead([dom, this](Ground::ULitVec &&lits) { return make_unique<Ground::HeadAggregateRule>(dom, std::move(lits)); },
        {[](Ground::ULitVec &, bool) { }, std::move(split)});
}
CreateHead LitHeadAggregate::toGround(ToGroundArg &, bool) const {
    throw std::runtime_error("Aggregate::rewriteAggregates must be called before LitAggregate::toGround");
}
CreateHead Disjunction::toGround(ToGroundArg &x, bool) const {
    Ground::SDisjunctionDomain dom(std::make_shared<Ground::DisjunctionDomain>(x.newId(*this)));
    CreateStmVec split;
    for (auto &y : elems) { 
        split.emplace_back([this, dom, &y, &x](Ground::ULitVec &&) -> Ground::UStm {
            Ground::ULitVec lits;
            for (auto &z : y.second) { lits.emplace_back(z->toGround(x.domains)); }
            lits.emplace_back(make_unique<Ground::DisjunctionLiteral>(dom));
            UTerm headRep(y.first->headRepr());
            PredicateDomain *pred = headRep ? &add(x.domains, headRep->getSig()) : nullptr;
            return make_unique<Ground::DisjunctionAccumulate>(dom, pred, std::move(headRep), std::move(lits));
        });
    }
    return
        CreateHead([dom, &x](Ground::ULitVec &&lits) { return make_unique<Ground::DisjunctionRule>(dom, std::move(lits)); },
        {[](Ground::ULitVec &, bool) { }, std::move(split)});
}
CreateHead SimpleHeadLiteral::toGround(ToGroundArg &x, bool external) const {
    return
        {[this, &x, external](Ground::ULitVec &&lits) -> Ground::UStm {
            if (UTerm headRepr = lit->headRepr()) {
                FWSignature sig(headRepr->getSig());
                return make_unique<Ground::Rule>(&add(x.domains, sig), std::move(headRepr), std::move(lits), external);
            }
            else {
                return make_unique<Ground::Rule>(nullptr, nullptr, std::move(lits), external);
            }
        },
        { [](Ground::ULitVec &, bool) { }, { } }};
}

// }}}

// {{{ definition of *BodyAggregate::isAssignment

bool TupleBodyAggregate::isAssignment() const { return bounds.size() == 1 && naf == NAF::POS && bounds.front().rel == Relation::ASSIGN && bounds.front().bound->getInvertibility() == Term::INVERTIBLE; }
bool LitBodyAggregate::isAssignment() const   { return false; }
bool Conjunction::isAssignment() const        { return false; }
bool SimpleBodyLiteral::isAssignment() const  { return false; }
bool DisjointAggregate::isAssignment() const  { return false; }

// }}}
// {{{ definition of *BodyAggregate::removeAssignment

void TupleBodyAggregate::removeAssignment() { 
    assert (isAssignment());
    bounds.front().rel = Relation::EQ;
}
void LitBodyAggregate::removeAssignment()  { }
void Conjunction::removeAssignment()       { }
void SimpleBodyLiteral::removeAssignment() { }
void DisjointAggregate::removeAssignment() { }

// }}}

// {{{ definition of TupleBodyAggregate

TupleBodyAggregate::TupleBodyAggregate(NAF naf, AggregateFunction fun, BoundVec &&bounds, BodyAggrElemVec &&elems)
    : naf(naf)
    , fun(fun)
    , bounds(std::move(bounds))
    , elems(std::move(elems)) { }

TupleBodyAggregate::~TupleBodyAggregate() { }

// }}}
// {{{ definition of LitBodyAggregate

LitBodyAggregate::LitBodyAggregate(NAF naf, AggregateFunction fun, BoundVec &&bounds, CondLitVec &&elems)
    : naf(naf)
    , fun(fun)
    , bounds(std::move(bounds))
    , elems(std::move(elems)) { }

LitBodyAggregate::~LitBodyAggregate() { }

// }}}
// {{{ definition of Conjunction

Conjunction::Conjunction(ULit &&head, ULitVec &&cond)
    : head(std::move(head))
    , cond(std::move(cond)) { }

Conjunction::~Conjunction() { }

// }}}
// {{{ definition of SimpleBodyLiteral

SimpleBodyLiteral::SimpleBodyLiteral(ULit &&lit)
    : lit(std::move(lit)) { }

Location const &SimpleBodyLiteral::loc() const { return lit->loc(); }
void SimpleBodyLiteral::loc(Location const &loc) { lit->loc(loc); }
SimpleBodyLiteral::~SimpleBodyLiteral() { }

// }}}

// {{{ definition of TupleHeadAggregate

TupleHeadAggregate::TupleHeadAggregate(AggregateFunction fun, BoundVec &&bounds, HeadAggrElemVec &&elems)
    : fun(fun)
    , bounds(std::move(bounds))
    , elems(std::move(elems)) { }

TupleHeadAggregate::~TupleHeadAggregate() { }

// }}}
// {{{ definition of LitHeadAggregate

LitHeadAggregate::LitHeadAggregate(AggregateFunction fun, BoundVec &&bounds, CondLitVec &&elems)
    : fun(fun)
    , bounds(std::move(bounds))
    , elems(std::move(elems)) { }

LitHeadAggregate::~LitHeadAggregate() { }

// }}}
// {{{ definition of Disjunction

Disjunction::Disjunction(CondLitVec &&elems)
    : elems(std::move(elems)) { }

Disjunction::~Disjunction() { }

// }}}
// {{{ definition of SimpleHeadLiteral

SimpleHeadLiteral::SimpleHeadLiteral(ULit &&lit)
    : lit(std::move(lit)) { }

Location const &SimpleHeadLiteral::loc() const { return lit->loc(); }
void SimpleHeadLiteral::loc(Location const &loc) { lit->loc(loc); }
SimpleHeadLiteral::~SimpleHeadLiteral() { }

// }}}
// {{{ definition of SimpleHeadLiteral

DisjointAggregate::DisjointAggregate(NAF naf, CSPElemVec &&elems)
    : naf(naf)
    , elems(std::move(elems)) { }

DisjointAggregate::~DisjointAggregate() { }

// }}}

// definition of CSPElem {{{

CSPElem::CSPElem(Location const &loc, UTermVec &&tuple, CSPAddTerm &&value, ULitVec &&cond)
    : loc(loc)
    , tuple(std::move(tuple))
    , value(std::move(value))
    , cond(std::move(cond)) { }
CSPElem::CSPElem(CSPElem &&) = default;
CSPElem::~CSPElem() { }
void CSPElem::print(std::ostream &out) const {
    using namespace std::placeholders;
    print_comma(out, tuple, ",", std::bind(&Term::print, _2, _1));
    out << ":" << value;
    if (!cond.empty()) {
        out << ":";
        print_comma(out, cond, ",", std::bind(&Literal::print, _2, _1));
    }
}
size_t CSPElem::hash() const {
    return get_value_hash(tuple, value, cond);
}
CSPElem CSPElem::clone() const {
    return {loc, get_clone(tuple), get_clone(value), get_clone(cond)};
}
bool CSPElem::operator==(CSPElem const &x) const {
    return is_value_equal_to(tuple, x.tuple) && is_value_equal_to(value, x.value) && is_value_equal_to(cond, x.cond);
}

// }}}


} } // namespace Input Gringo

