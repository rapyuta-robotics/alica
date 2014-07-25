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

#ifndef _GRINGO_OUTPUT_LITERALS_HH
#define _GRINGO_OUTPUT_LITERALS_HH

#include <gringo/terms.hh>
#include <gringo/domain.hh>
#include <gringo/intervals.hh>
#include <gringo/unique_list.hh>
#include <gringo/output/literal.hh>

namespace Gringo { namespace Output {

// {{{ declaration of AuxLiteral

struct AuxAtom {
    AuxAtom(unsigned name);
    int lparseUid(LparseOutputter &out);
    unsigned name;
    unsigned uid = 0;
};
using SAuxAtom    = std::shared_ptr<AuxAtom>;
using SAuxAtomVec = std::vector<SAuxAtom>;

std::ostream &operator<<(std::ostream &out, AuxAtom const &x);

struct AuxLiteral : Literal {
    AuxLiteral(SAuxAtom atom, bool negative);
    virtual AuxLiteral *clone() const;
    virtual ULit toLparse(LparseTranslator &x);
    virtual void printPlain(std::ostream &out) const;
    virtual bool isIncomplete() const;
    virtual int lparseUid(LparseOutputter &out) const;
    virtual void makeEqual(ULit &&lit, LparseTranslator &x) const;
    virtual size_t hash() const;
    virtual bool operator==(Literal const &x) const;
    virtual ~AuxLiteral();
    
    SAuxAtom atom;
    bool     negative;
};

// }}}
// {{{ declaration of BooleanLiteral

struct BooleanLiteral : Literal {
    BooleanLiteral(bool value);
    virtual BooleanLiteral *clone() const;
    virtual ULit toLparse(LparseTranslator &x);
    virtual void printPlain(std::ostream &out) const;
    virtual bool isIncomplete() const;
    virtual int lparseUid(LparseOutputter &out) const;
    virtual void makeEqual(ULit &&lit, LparseTranslator &x) const;
    virtual size_t hash() const;
    virtual bool operator==(Literal const &x) const;
    virtual ~BooleanLiteral();

    bool value;
};

// }}}
// {{{ declaration of PredicateLiteral

struct PredicateLiteral : Literal {
    PredicateLiteral();
    PredicateLiteral(NAF naf, PredicateDomain::element_type &repr);
    virtual void printPlain(std::ostream &out) const;
    virtual bool isIncomplete() const;
    virtual PredicateLiteral *clone() const;
    virtual ULit toLparse(LparseTranslator &x);
    virtual void makeEqual(ULit &&lit, LparseTranslator &x) const;
    virtual int lparseUid(LparseOutputter &out) const;
    virtual size_t hash() const;
    virtual bool operator==(Literal const &x) const;
    bool invertible() const;
    void invert();
    virtual ~PredicateLiteral();

    NAF                            naf  = NAF::POS;
    PredicateDomain::element_type *repr = nullptr;
};

// }}}
// {{{ declaration of BodyAggregateState

int clamp(int64_t x);
bool neutral(ValVec const &tuple, AggregateFunction fun, Location const &loc);
int toInt(IntervalSet<Value>::LBound const &x);
int toInt(IntervalSet<Value>::RBound const &x);
Value getWeight(AggregateFunction fun, FWValVec const &x);

using BdAggrElemSet = unique_list<std::pair<FWValVec, std::vector<ULitVec>>, extract_first<FWValVec>>;

struct BodyAggregateState {
    enum State { UNKNOWN, DEFINED, OPEN };
    using Bounds       = IntervalSet<Value>;
    using element_type = std::pair<Value const, BodyAggregateState>;

    bool fact(bool recursive) const;
    unsigned generation() const;
    void generation(unsigned x);
    bool isFalse();
    static element_type &ignore();
    void accumulate(ValVec const &tuple, AggregateFunction fun, bool fact, bool remove);
    void init(AggregateFunction fun);
    bool defined() const;
    Bounds::Interval range(AggregateFunction fun) const;
    ~BodyAggregateState();

    Bounds         bounds;
    BdAggrElemSet  elems;
    union {
        int64_t    intMin;
        Value::POD valMin;
    };
    union {
        int64_t    intMax;
        Value::POD valMax;
    };
    unsigned _generation = 0;
    State    state       = OPEN;
    bool     _positive   = false;
    bool     _fact       = false;
};

// }}}
// {{{ declaration of BodyAggregate

struct BodyAggregate : Literal {
    using BoundsVec = std::vector<std::pair<Relation, Value>>;
    BodyAggregate(Location const *&loc);
    virtual void printPlain(std::ostream &out) const;
    virtual bool isIncomplete() const;
    virtual BodyAggregate *clone() const;
    virtual size_t hash() const;
    virtual bool operator==(Literal const &) const;
    virtual ULit toLparse(LparseTranslator &x);
    virtual void makeEqual(ULit &&lit, LparseTranslator &x) const;
    virtual int lparseUid(LparseOutputter &out) const;
    virtual ~BodyAggregate();

    Location const                  *&loc;
    BoundsVec                         bounds;
    NAF                               naf        = NAF::POS;
    AggregateFunction                 fun        = AggregateFunction::COUNT;
    bool                              incomplete = false;
    BodyAggregateState::element_type *repr       = nullptr;
};

// }}}
// {{{ declaration of AssignmentAggregateState

struct AssignmentAggregateState {
    using ValSet = std::set<Value>;
    struct Data {
        BdAggrElemSet elems;
        unsigned      offset = 0;
        bool          fact   = false;
    };
    using element_type = std::pair<Value const, AssignmentAggregateState>;

    AssignmentAggregateState(Data *data = nullptr, unsigned generation = 0);
    bool fact(bool recursive) const;
    unsigned generation() const;
    void generation(unsigned x);
    bool isFalse();
    static element_type &ignore();
    bool defined() const;

    Data      *data; // can be a reference!!!
    unsigned  _generation;
};

// }}}
// {{{ declaration of AssignmentAggregate

struct AssignmentAggregate : Literal {
    AssignmentAggregate(Location const *&loc);
    virtual void printPlain(std::ostream &out) const;
    virtual bool isIncomplete() const;
    virtual AssignmentAggregate *clone() const;
    virtual size_t hash() const;
    virtual bool operator==(Literal const &) const;
    virtual ULit toLparse(LparseTranslator &x);
    virtual void makeEqual(ULit &&lit, LparseTranslator &x) const;
    virtual int lparseUid(LparseOutputter &out) const;
    virtual ~AssignmentAggregate();

    Location const                        *&loc;
    AggregateFunction                       fun        = AggregateFunction::COUNT;
    bool                                    incomplete = false;
    AssignmentAggregateState::element_type *repr       = nullptr;
};

// }}}
// {{{ declaration of ConjunctionElem

struct ConjunctionElem {
    ConjunctionElem(PredicateDomain::element_type* head, ULitVec &&body);
    size_t hash() const;
    bool operator==(ConjunctionElem const &x) const;
    void print(std::ostream &out) const;

    PredicateDomain::element_type* head;
    ULitVec                        body;
};

inline std::ostream &operator<<(std::ostream &out, ConjunctionElem const &x) {
    x.print(out);
    return out;
}

// }}}
// {{{ declaration of ConjunctionState

struct ConjunctionState {
    using element_type = std::pair<Value const, ConjunctionState>;
    using Elem         = ConjunctionElem;
    using ElemSet      = unique_list<Elem, identity<Elem>, call_hash<Elem>>;
    using BlockSet     = std::unordered_set<Value>;

    bool fact(bool recursive) const;
    unsigned generation() const;
    void generation(unsigned x);
    bool isFalse();
    static element_type &ignore();
    bool defined() const;

    BlockSet blocked;
    ElemSet  elems;
    SAuxAtom bdLit;
    bool     _fact       = true;
    unsigned _generation = 0; // 0 = undefined, 1 = enqueued
};

// }}}
// {{{ declaration of Conjunction

struct Conjunction : Literal {
    virtual void printPlain(std::ostream &out) const;
    virtual bool isIncomplete() const;
    virtual Conjunction *clone() const;
    virtual size_t hash() const;
    virtual bool operator==(Literal const &) const;
    virtual ULit toLparse(LparseTranslator &x);
    virtual void makeEqual(ULit &&lit, LparseTranslator &x) const;
    virtual int lparseUid(LparseOutputter &out) const;
    virtual ~Conjunction();

    bool                            incomplete = false;
    ConjunctionState::element_type *repr       = nullptr;
};

// }}}
// {{{ declaration of DisjointState

struct DisjointElem {
    DisjointElem(CSPGroundAdd &&value, int fixed, ULitVec &&lits);
    DisjointElem(DisjointElem &&);
    DisjointElem clone() const;
    ~DisjointElem();

    CSPGroundAdd value;
    int fixed;
    ULitVec lits;
};

using DisjointElemVec = std::vector<DisjointElem>;
using DisjointElemSet = unique_list<std::pair<FWValVec, DisjointElemVec>, extract_first<FWValVec>>;

struct DisjointState {
    using element_type = std::pair<Value const, DisjointState>;

    bool fact(bool recursive) const;
    unsigned generation() const;
    void generation(unsigned x);
    bool isFalse();
    static element_type &ignore();
    bool defined() const;
    ~DisjointState();

    DisjointElemSet elems;
    unsigned _generation = 0;
};

// }}}
// {{{ declaration of DisjointLiteral

struct DisjointLiteral : Literal {
    DisjointLiteral(NAF naf);
    virtual void printPlain(std::ostream &out) const;
    virtual bool isIncomplete() const;
    virtual DisjointLiteral *clone() const;
    virtual size_t hash() const;
    virtual bool operator==(Literal const &) const;
    virtual ULit toLparse(LparseTranslator &x);
    virtual void makeEqual(ULit &&lit, LparseTranslator &x) const;
    virtual int lparseUid(LparseOutputter &out) const;
    virtual ~DisjointLiteral();

    NAF const                    naf;
    bool                         incomplete = false;
    DisjointState::element_type *repr       = nullptr;
};

// }}}
// {{{ declaration of CSPLiteral

struct CSPLiteral : Literal {
    CSPLiteral();
    void reset(CSPGroundLit &&ground);
    static void printElem(std::ostream &out, ConjunctionState::Elem const &x);
    virtual void printPlain(std::ostream &out) const;
    virtual bool isIncomplete() const;
    virtual CSPLiteral *clone() const;
    virtual size_t hash() const;
    virtual bool operator==(Literal const &) const;
    virtual ULit toLparse(LparseTranslator &x);
    virtual void makeEqual(ULit &&lit, LparseTranslator &x) const;
    virtual int lparseUid(LparseOutputter &out) const;
    virtual bool isBound(Value &value, bool negate) const;
    virtual void updateBound(CSPBound &bounds, bool negate) const;
    virtual ~CSPLiteral();
    CSPGroundLit ground;
};

// }}}

} } // namespace Output Gringo

GRINGO_CALL_CLONE(Gringo::Output::DisjointElem)

#endif // _GRINGO_OUTPUT_LITERALS_HH

