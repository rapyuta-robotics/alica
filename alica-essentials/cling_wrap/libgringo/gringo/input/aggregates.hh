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

#ifndef _GRINGO_INPUT_AGGREGATES_HH
#define _GRINGO_INPUT_AGGREGATES_HH

#include <gringo/input/aggregate.hh>
#include <gringo/terms.hh>

namespace Gringo { namespace Input {

// {{{ declaration of TupleBodyAggregate

struct TupleBodyAggregate : BodyAggregate {
    TupleBodyAggregate(NAF naf, AggregateFunction fun, BoundVec &&bounds, BodyAggrElemVec &&elems);
    virtual bool rewriteAggregates(UBodyAggrVec &aggr);
    virtual bool isAssignment() const;
    virtual void removeAssignment();
    virtual void collect(VarTermBoundVec &vars) const;
    virtual bool operator==(BodyAggregate const &other) const;
    virtual void print(std::ostream &out) const;
    virtual size_t hash() const;
    virtual TupleBodyAggregate *clone() const;
    virtual void unpool(UBodyAggrVec &x, bool beforeRewrite);
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    virtual void assignLevels(AssignLevel &lvl);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &assign, unsigned &auxNum);
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check(ChkLvlVec &lvl) const;
    virtual void replace(Defines &dx);
    virtual CreateBody toGround(ToGroundArg &x, Ground::UStmVec &stms) const;
    virtual ~TupleBodyAggregate();

    NAF naf;
    AggregateFunction fun;
    BoundVec bounds;
    BodyAggrElemVec elems;
};

// }}}
// {{{ declaration of LitBodyAggregate

struct LitBodyAggregate : BodyAggregate {
    LitBodyAggregate(NAF naf, AggregateFunction fun, BoundVec &&bounds, CondLitVec &&elems);
    virtual bool rewriteAggregates(UBodyAggrVec &aggr);
    virtual bool isAssignment() const;
    virtual void removeAssignment();
    virtual void collect(VarTermBoundVec &vars) const;
    virtual bool operator==(BodyAggregate const &other) const;
    virtual void print(std::ostream &out) const;
    virtual size_t hash() const;
    virtual LitBodyAggregate *clone() const;
    virtual void unpool(UBodyAggrVec &x, bool beforeRewrite);
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    virtual void assignLevels(AssignLevel &lvl);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &assign, unsigned &auxNum);
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check(ChkLvlVec &lvl) const;
    virtual void replace(Defines &dx);
    virtual CreateBody toGround(ToGroundArg &x, Ground::UStmVec &stms) const;
    virtual ~LitBodyAggregate();

    NAF naf;
    AggregateFunction fun;
    BoundVec bounds;
    CondLitVec elems;
};

// }}}
// {{{ declaration of Conjunction

struct Conjunction : BodyAggregate {
    Conjunction(ULit &&head, ULitVec &&cond);
    virtual bool rewriteAggregates(UBodyAggrVec &aggr);
    virtual bool isAssignment() const;
    virtual void removeAssignment();
    virtual void collect(VarTermBoundVec &vars) const;
    virtual bool operator==(BodyAggregate const &other) const;
    virtual void print(std::ostream &out) const;
    virtual size_t hash() const;
    virtual Conjunction *clone() const;
    virtual void unpool(UBodyAggrVec &x, bool beforeRewrite);
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    virtual void assignLevels(AssignLevel &lvl);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &assign, unsigned &auxNum);
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check(ChkLvlVec &lvl) const;
    virtual void replace(Defines &dx);
    virtual CreateBody toGround(ToGroundArg &x, Ground::UStmVec &stms) const;
    virtual ~Conjunction();

    ULit head;
    ULitVec cond;
};

// }}}
// {{{ declaration of SimpleBodyLiteral

struct SimpleBodyLiteral : BodyAggregate {
    SimpleBodyLiteral(ULit &&lit);
    virtual Location const &loc() const;
    virtual void loc(Location const &loc);
    virtual bool rewriteAggregates(UBodyAggrVec &aggr);
    virtual void removeAssignment();
    virtual bool isAssignment() const;
    virtual void collect(VarTermBoundVec &vars) const;
    virtual bool operator==(BodyAggregate const &other) const;
    virtual void print(std::ostream &out) const;
    virtual size_t hash() const;
    virtual SimpleBodyLiteral *clone() const;
    virtual void unpool(UBodyAggrVec &x, bool beforeRewrite);
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    virtual void assignLevels(AssignLevel &lvl);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &assign, unsigned &auxNum);
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check(ChkLvlVec &lvl) const;
    virtual void replace(Defines &dx);
    virtual CreateBody toGround(ToGroundArg &x, Ground::UStmVec &stms) const;
    virtual ~SimpleBodyLiteral();

    ULit lit;
};

// }}}
// {{{ declaration of DisjointAggregate

struct CSPElem {
    CSPElem(Location const &loc, UTermVec &&tuple, CSPAddTerm &&value, ULitVec &&cond);
    CSPElem(CSPElem &&x);
    ~CSPElem();
    void print(std::ostream &out) const;
    CSPElem clone() const;
    size_t hash() const;
    bool operator==(CSPElem const &other) const;

    Location   loc;
    UTermVec   tuple;
    CSPAddTerm value;
    ULitVec    cond;
};

inline std::ostream &operator<<(std::ostream &out, CSPElem const &x) {
    x.print(out);
    return out;
}

using CSPElemVec = std::vector<CSPElem>;

struct DisjointAggregate : BodyAggregate {
    DisjointAggregate(NAF naf, CSPElemVec &&elems);
    virtual bool rewriteAggregates(UBodyAggrVec &aggr);
    virtual bool isAssignment() const;
    virtual void removeAssignment();
    virtual void collect(VarTermBoundVec &vars) const;
    virtual bool operator==(BodyAggregate const &other) const;
    virtual void print(std::ostream &out) const;
    virtual size_t hash() const;
    virtual DisjointAggregate *clone() const;
    virtual void unpool(UBodyAggrVec &x, bool beforeRewrite);
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    virtual void assignLevels(AssignLevel &lvl);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, Literal::AssignVec &assign, unsigned &auxNum);
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check(ChkLvlVec &lvl) const;
    virtual void replace(Defines &dx);
    virtual CreateBody toGround(ToGroundArg &x, Ground::UStmVec &stms) const;
    virtual ~DisjointAggregate();

    NAF        naf;
    CSPElemVec elems;
};

// }}}

// {{{ declaration of TupleHeadAggregate

struct TupleHeadAggregate : HeadAggregate {
    TupleHeadAggregate(AggregateFunction fun, BoundVec &&bounds, HeadAggrElemVec &&elems);
    virtual UHeadAggr rewriteAggregates(UBodyAggrVec &aggr);
    virtual void collect(VarTermBoundVec &vars) const;
    virtual bool operator==(HeadAggregate const &other) const;
    virtual void print(std::ostream &out) const;
    virtual size_t hash() const;
    virtual TupleHeadAggregate *clone() const;
    virtual void unpool(UHeadAggrVec &x, bool beforeRewrite);
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    virtual void assignLevels(AssignLevel &lvl);
    virtual UHeadAggr shiftHead(UBodyAggrVec &aggr);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check(ChkLvlVec &lvl) const;
    virtual void replace(Defines &dx);
    virtual CreateHead toGround(ToGroundArg &x, bool external) const;
    virtual ~TupleHeadAggregate();

    AggregateFunction fun;
    BoundVec bounds;
    HeadAggrElemVec elems;
};

// }}}
// {{{ declaration of LitHeadAggregate

struct LitHeadAggregate : HeadAggregate {
    LitHeadAggregate(AggregateFunction fun, BoundVec &&bounds, CondLitVec &&elems);
    virtual UHeadAggr shiftHead(UBodyAggrVec &aggr);
    virtual UHeadAggr rewriteAggregates(UBodyAggrVec &aggr);
    virtual void collect(VarTermBoundVec &vars) const;
    virtual bool operator==(HeadAggregate const &other) const;
    virtual void print(std::ostream &out) const;
    virtual size_t hash() const;
    virtual LitHeadAggregate *clone() const;
    virtual void unpool(UHeadAggrVec &x, bool beforeRewrite);
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    virtual void assignLevels(AssignLevel &lvl);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check(ChkLvlVec &lvl) const;
    virtual void replace(Defines &dx);
    virtual CreateHead toGround(ToGroundArg &x, bool external) const;
    virtual ~LitHeadAggregate();

    AggregateFunction fun;
    BoundVec bounds;
    CondLitVec elems;
};

// }}}
// {{{ declaration of Disjunction

struct Disjunction : HeadAggregate {
    Disjunction(CondLitVec &&elems);
    virtual UHeadAggr shiftHead(UBodyAggrVec &aggr);
    virtual UHeadAggr rewriteAggregates(UBodyAggrVec &aggr);
    virtual void collect(VarTermBoundVec &vars) const;
    virtual bool operator==(HeadAggregate const &other) const;
    virtual void print(std::ostream &out) const;
    virtual size_t hash() const;
    virtual Disjunction *clone() const;
    virtual void unpool(UHeadAggrVec &x, bool beforeRewrite);
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    virtual void assignLevels(AssignLevel &lvl);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check(ChkLvlVec &lvl) const;
    virtual void replace(Defines &dx);
    virtual CreateHead toGround(ToGroundArg &x, bool external) const;
    virtual ~Disjunction();

    CondLitVec elems;
};

// }}}
// {{{ declaration of SimpleHeadLiteral

struct SimpleHeadLiteral : HeadAggregate {
    SimpleHeadLiteral(ULit &&lit);
    virtual Location const &loc() const;
    virtual void loc(Location const &loc);
    virtual UHeadAggr shiftHead(UBodyAggrVec &aggr);
    virtual UHeadAggr rewriteAggregates(UBodyAggrVec &aggr);
    virtual void collect(VarTermBoundVec &vars) const;
    virtual bool operator==(HeadAggregate const &other) const;
    virtual void print(std::ostream &out) const;
    virtual size_t hash() const;
    virtual SimpleHeadLiteral *clone() const;
    virtual void unpool(UHeadAggrVec &x, bool beforeRewrite);
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    virtual void assignLevels(AssignLevel &lvl);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum);
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check(ChkLvlVec &lvl) const;
    virtual void replace(Defines &dx);
    virtual CreateHead toGround(ToGroundArg &x, bool external) const;
    virtual Value isEDB() const;
    virtual ~SimpleHeadLiteral();

    ULit lit;
};

// }}}

} } // namespace Input Gringo

GRINGO_CALL_HASH(Gringo::Input::CSPElem)
GRINGO_CALL_CLONE(Gringo::Input::CSPElem)

#endif // _GRINGO_INPUT_AGGREGATES_HH

