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

#ifndef _GRINGO_OUTPUT_LITERAL_HH
#define _GRINGO_OUTPUT_LITERAL_HH

#include <gringo/clonable.hh>
#include <gringo/comparable.hh>
#include <gringo/hashable.hh>
#include <gringo/value.hh>

namespace Gringo { namespace Output {

struct LparseOutputter;
struct LparseTranslator;
struct Statement;
using UStm     = std::unique_ptr<Statement>;
using UStmVec  = std::vector<UStm>;
using CSPBound = std::pair<int, int>;

// {{{ declaration of Literal

struct Literal;
using ULit = std::unique_ptr<Literal>;
struct Literal : Clonable<Literal>, Hashable, Comparable<Literal> {
    virtual ULit toLparse(LparseTranslator &x) = 0;
    virtual void makeEqual(ULit &&lit, LparseTranslator &x) const = 0;
    virtual void printPlain(std::ostream &out) const = 0;
    virtual bool isIncomplete() const = 0;
    virtual int lparseUid(LparseOutputter &out) const = 0;
    virtual bool isBound(Value &value, bool negate) const { (void)value; (void)negate; return false; }
    virtual void updateBound(CSPBound &bounds, bool negate) const { (void)bounds; (void)negate; }
    virtual bool invertible() const { return false; }
    virtual void invert() { assert(false); }
    virtual ~Literal() { }
};
using LitVec  = std::vector<std::reference_wrapper<Literal>>;
using ULitVec = std::vector<ULit>;

// }}}

} } // namespace Output Gringo

GRINGO_HASH(Gringo::Output::Literal)

#endif // _GRINGO_OUTPUT_LITERAL_HH

