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

#ifndef _GRINGO_INPUT_LITERALS_HH
#define _GRINGO_INPUT_LITERALS_HH

#include <gringo/terms.hh>
#include <gringo/input/literal.hh>

namespace Gringo { namespace Input {

// {{{ declaration of PredicateLiteral

struct PredicateLiteral : Literal {
    PredicateLiteral(NAF naf, UTerm &&repr);
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void toTuple(UTermVec &tuple, int &id);
    virtual PredicateLiteral *clone() const;
    virtual void print(std::ostream &out) const;
    virtual bool operator==(Literal const &other) const;
    virtual size_t hash() const;
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum, bool positional = true);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, AssignVec &assign, unsigned &auxNum);
    virtual ULitVec unpool(bool beforeRewrite) const;
    virtual Value isEDB() const;
    virtual bool hasPool(bool beforeRewrite) const;
    virtual void replace(Defines &dx);
    virtual Ground::ULit toGround(PredDomMap &x) const;
    virtual ULit shift(bool negate);
    virtual UTerm headRepr() const;
    virtual ~PredicateLiteral();

    NAF naf;
    UTerm repr;
};

struct ProjectionLiteral : PredicateLiteral {
    ProjectionLiteral(UTerm &&repr);
    virtual ProjectionLiteral *clone() const;
    virtual ULitVec unpool(bool beforeRewrite) const;
    virtual Ground::ULit toGround(PredDomMap &x) const;
    virtual ULit shift(bool negate);
    virtual ~ProjectionLiteral();
};

// }}}
// {{{ declaration of RelationLiteral

struct RelationLiteral : Literal {
    RelationLiteral(Relation rel, UTerm &&left, UTerm &&right);
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void toTuple(UTermVec &tuple, int &id);
    virtual RelationLiteral *clone() const;
    virtual void print(std::ostream &out) const;
    virtual bool operator==(Literal const &other) const;
    virtual size_t hash() const;
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum, bool positional = true);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, AssignVec &assign, unsigned &auxNum);
    virtual ULitVec unpool(bool beforeRewrite) const;
    virtual bool hasPool(bool beforeRewrite) const;
    virtual void replace(Defines &dx);
    virtual Ground::ULit toGround(PredDomMap &x) const;
    virtual UTerm headRepr() const;
    virtual ULit shift(bool negate);
    virtual ~RelationLiteral();
    static ULit make(Term::ArithmeticsMap::value_type::value_type &x);
    static ULit make(Literal::AssignVec::value_type &x);

    Relation rel;
    UTerm left;
    UTerm right;
};

// }}}
// {{{ declaration of RangeLiteral

struct RangeLiteral : Literal {
    RangeLiteral(UTerm &&assign, UTerm &&lower, UTerm &&upper);
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void toTuple(UTermVec &tuple, int &id);
    virtual RangeLiteral *clone() const;
    virtual void print(std::ostream &out) const;
    virtual bool operator==(Literal const &other) const;
    virtual size_t hash() const;
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum, bool positional = true);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, AssignVec &assign, unsigned &auxNum);
    virtual ULitVec unpool(bool beforeRewrite) const;
    virtual bool hasPool(bool beforeRewrite) const;
    virtual void replace(Defines &dx);
    virtual Ground::ULit toGround(PredDomMap &x) const;
    virtual ULit shift(bool negate);
    virtual UTerm headRepr() const;
    virtual ~RangeLiteral();
    static ULit make(Term::DotsMap::value_type &dot);

    UTerm assign;
    UTerm lower;
    UTerm upper;
};

// }}}
// {{{ declaration of ScriptLiteral

struct ScriptLiteral : Literal {
    ScriptLiteral(UTerm &&assign, FWString name, UTermVec &&args);
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void toTuple(UTermVec &tuple, int &id);
    virtual ScriptLiteral *clone() const;
    virtual void print(std::ostream &out) const;
    virtual bool operator==(Literal const &other) const;
    virtual size_t hash() const;
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum, bool positional = true);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, AssignVec &assign, unsigned &auxNum);
    virtual ULitVec unpool(bool beforeRewrite) const;
    virtual bool hasPool(bool beforeRewrite) const;
    virtual void replace(Defines &dx);
    virtual Ground::ULit toGround(PredDomMap &x) const;
    virtual ULit shift(bool negate);
    virtual UTerm headRepr() const;
    virtual ~ScriptLiteral();
    static ULit make(Term::ScriptMap::value_type &script);

    UTerm assign;
    FWString name;
    UTermVec args;
};

// }}}
// {{{ declaration of FalseLiteral

struct FalseLiteral : Literal {
    FalseLiteral();
    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void toTuple(UTermVec &tuple, int &id);
    virtual FalseLiteral *clone() const;
    virtual void print(std::ostream &out) const;
    virtual bool operator==(Literal const &other) const;
    virtual size_t hash() const;
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum, bool positional = true);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, AssignVec &assign, unsigned &auxNum);
    virtual ULitVec unpool(bool beforeRewrite) const;
    virtual bool hasPool(bool beforeRewrite) const;
    virtual void replace(Defines &dx);
    virtual Ground::ULit toGround(PredDomMap &x) const;
    virtual ULit shift(bool negate);
    virtual UTerm headRepr() const;
    virtual ~FalseLiteral();
};

// }}}
// {{{ declaration of CSPLiteral

struct CSPLiteral : Literal {
    using Terms = std::vector<CSPRelTerm>;

    CSPLiteral(Relation rel, CSPAddTerm &&left, CSPAddTerm &&right);
    CSPLiteral(Terms &&terms);
    void append(Relation rel, CSPAddTerm &&x);

    virtual void collect(VarTermBoundVec &vars, bool bound) const;
    virtual void toTuple(UTermVec &tuple, int &id);
    virtual CSPLiteral *clone() const;
    virtual void print(std::ostream &out) const;
    virtual bool operator==(Literal const &other) const;
    virtual size_t hash() const;
    virtual void simplify(Projections &project, Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum, bool positional = true);
    virtual void rewriteArithmetics(Term::ArithmeticsMap &arith, AssignVec &assign, unsigned &auxNum);
    virtual ULitVec unpool(bool beforeRewrite) const;
    virtual bool hasPool(bool beforeRewrite) const;
    virtual void replace(Defines &dx);
    virtual Ground::ULit toGround(PredDomMap &x) const;
    virtual ULit shift(bool negate);
    virtual UTerm headRepr() const;
    virtual ~CSPLiteral();

    Terms terms;
};
using UCSPLit = std::unique_ptr<CSPLiteral>;

// }}}

} } // namespace Input Gringo

#endif // _GRINGO_INPUT_LITERALS_HH

