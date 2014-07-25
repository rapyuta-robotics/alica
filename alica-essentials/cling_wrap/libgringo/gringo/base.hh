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

#ifndef _GRINGO_BASE_HH
#define _GRINGO_BASE_HH

#include <cassert>
#include <gringo/term.hh>
#include <deque>

namespace Gringo {

// {{{ declaration of NAF

enum class NAF { POS = 0, NOT = 1, NOTNOT = 2 };
std::ostream &operator<<(std::ostream &out, NAF naf);
NAF inv(NAF naf);

enum class RECNAF { POS, NOT, NOTNOT, RECNOT };
RECNAF recnaf(NAF naf, bool recursive);
std::ostream &operator<<(std::ostream &out, RECNAF naf);


// }}}
// {{{ declaration of Relation

enum class Relation : unsigned { GT, LT, LEQ, GEQ, EQ, NEQ, ASSIGN };

Relation inv(Relation rel);
Relation neg(Relation rel);

std::ostream &operator<<(std::ostream &out, Relation rel);

// }}}
// {{{ declaration of AggregateFunction

enum class AggregateFunction : unsigned { COUNT, SUM, SUMP, MIN, MAX };

std::ostream &operator<<(std::ostream &out, AggregateFunction fun);

// }}}
// {{{ declaration of Bound

struct Bound;
typedef std::vector<Bound> BoundVec;

struct Bound {
    Bound(Relation rel, UTerm &&bound);
    Bound(Bound &&bound) noexcept;
    Bound& operator=(Bound &&) noexcept;
    size_t hash() const;
    bool operator==(Bound const &other) const;
    //! Unpool the terms in the bound.
    BoundVec unpool();
    //! Simplify the terms in the bound.
    //! \pre Must be called after unpool.
    void simplify(Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum);
    //! Rewrite arithmetics.
    //! \pre Must be called after assignLevels.
    void rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum);

    Relation rel;
    UTerm bound;
};

// }}}

// {{{ definition of NAF

inline NAF inv(NAF naf) {
    switch (naf) {
        case NAF::NOTNOT: { return NAF::NOT; }
        case NAF::NOT:    { return NAF::NOTNOT; }
        case NAF::POS:    { return NAF::NOT; }
    }
    assert(false);
    return NAF::POS;
}

inline std::ostream &operator<<(std::ostream &out, NAF naf) {
    switch (naf) {
        case NAF::NOTNOT: { out << "not "; }
        case NAF::NOT:    { out << "not "; }
        case NAF::POS:    { }
    }
    return out;
}

inline std::ostream &operator<<(std::ostream &out, RECNAF naf) {
    switch (naf) {
        case RECNAF::NOTNOT: { out << "not "; }
        case RECNAF::RECNOT:
        case RECNAF::NOT:    { out << "not "; }
        case RECNAF::POS:    { }
    }
    return out;
}

inline RECNAF recnaf(NAF naf, bool recursive) {
    switch (naf) {
        case NAF::POS:    { return RECNAF::POS; }
        case NAF::NOT:    { return recursive ? RECNAF::RECNOT : RECNAF::NOT; }
        case NAF::NOTNOT: { return recursive ? RECNAF::NOTNOT : RECNAF::POS; }
    }
    return RECNAF::POS;
}

// }}}
// {{{ definition of Relation

inline std::ostream &operator<<(std::ostream &out, Relation rel) {
    switch (rel) {
        case Relation::GT:     { out << ">"; break; }
        case Relation::LT:     { out << "<"; break; }
        case Relation::LEQ:    { out << "<="; break; }
        case Relation::GEQ:    { out << ">="; break; }
        case Relation::EQ:     { out << "=="; break; }
        case Relation::NEQ:    { out << "!="; break; }
        case Relation::ASSIGN: { out << "="; break; }
    }
    return out;
}

inline Relation inv(Relation rel) {
    switch (rel) {
        case Relation::EQ:     { return Relation::EQ; }
        case Relation::NEQ:    { return Relation::NEQ; }
        case Relation::GEQ:    { return Relation::LEQ; }
        case Relation::LEQ:    { return Relation::GEQ; }
        case Relation::ASSIGN: { return Relation::ASSIGN; }
        case Relation::LT:     { return Relation::GT; }
        case Relation::GT:     { return Relation::LT; }
    }
    assert(false);
    return Relation(-1);
}

inline Relation neg(Relation rel) {
    switch (rel) {
        case Relation::EQ:     { return Relation::NEQ; }
        case Relation::NEQ:    { return Relation::EQ; }
        case Relation::GEQ:    { return Relation::LT; }
        case Relation::LEQ:    { return Relation::GT; }
        case Relation::ASSIGN: { return Relation::NEQ; }
        case Relation::LT:     { return Relation::GEQ; }
        case Relation::GT:     { return Relation::LEQ; }
    }
    assert(false);
    return Relation(-1);
}

// }}}
// {{{ definition of AggregateFunction

inline std::ostream &operator<<(std::ostream &out, AggregateFunction fun) {
    switch (fun) {
        case AggregateFunction::MIN:   { out << "#min"; break; }
        case AggregateFunction::MAX:   { out << "#max"; break; }
        case AggregateFunction::SUM:   { out << "#sum"; break; }
        case AggregateFunction::SUMP:  { out << "#sum+"; break; }
        case AggregateFunction::COUNT: { out << "#count"; break; }
    }
    return out;
}

// }}}
// {{{ definition of Bound

inline Bound::Bound(Relation rel, UTerm &&bound)
    : rel(rel)
    , bound(std::move(bound)) { }

inline size_t Bound::hash() const {
    return get_value_hash(size_t(rel), bound);
}

inline bool Bound::operator==(Bound const &other) const {
    return rel == other.rel && is_value_equal_to(bound, other.bound);
}

inline Bound::Bound(Bound &&) noexcept = default;

inline Bound& Bound::operator=(Bound &&) noexcept = default;

inline BoundVec Bound::unpool() {
    BoundVec pool;
    auto f = [&](UTerm &&x) { pool.emplace_back(rel, std::move(x)); };
    Term::unpool(bound, Gringo::unpool, f);
    return std::move(pool);
}

inline void Bound::simplify(Term::DotsMap &dots, Term::ScriptMap &scripts, unsigned &auxNum) {
    bound->simplify(dots, scripts, auxNum, false, false).update(bound);
}

inline void Bound::rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum) {
    if (rel == Relation::ASSIGN) { Term::replace(bound, bound->rewriteArithmetics(arith, auxNum)); }
}

template <>
struct clone<Bound> {
    Bound operator()(Bound const &bound) const {
        return Bound(bound.rel, get_clone(bound.bound));
    }
};

// }}}
 
} // namespace Gringo
 
namespace std {

// {{{ definition of hash<Gringo::Bound>

template <>
struct hash<Gringo::Bound> {
    size_t operator()(Gringo::Bound const &x) const { return x.hash(); }
};

// }}}
// {{{ definition of hash<Gringo::Relation>

template <>
struct hash<Gringo::Relation> {
    size_t operator()(Gringo::Relation const &x) const { return static_cast<unsigned>(x); }
};

// }}}
// {{{ definition of hash<Gringo::NAF>

template <>
struct hash<Gringo::NAF> {
    size_t operator()(Gringo::NAF const &x) const { return static_cast<unsigned>(x); }
};

// }}}

} // namespace std

#endif // _GRINGO_BASE_HH

