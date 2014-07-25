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

#ifndef _GRINGO_INPUT_STATEMENT_HH
#define _GRINGO_INPUT_STATEMENT_HH

#include <gringo/printable.hh>
#include <gringo/locatable.hh>

namespace Gringo { 
    
class Defines;
struct Term;

namespace Ground {

// {{{ forward declarations

struct Statement;
using UStm = std::unique_ptr<Statement>;
using UStmVec = std::vector<UStm>;

// }}}

} // namespace Ground

namespace Input {

// {{{ forward declarations

struct Literal;
using ULit = std::unique_ptr<Literal>;

struct BodyAggregate;
using UBodyAggr = std::unique_ptr<BodyAggregate>;
using UBodyAggrVec = std::vector<UBodyAggr>;

struct ToGroundArg;
struct Statement;
struct Projections;

// }}}
// {{{ declaration of Statement


using UStm = std::unique_ptr<Statement>;
using UStmVec = std::vector<UStm>;

struct Statement : Printable, Locatable {
    typedef std::vector<std::pair<ULit, UBodyAggrVec>> SplitVec;

    virtual Value isEDB() const;
    virtual void add(ULit &&lit) = 0;
    virtual void rewrite1(Projections &project) = 0;
    virtual void rewrite2(SplitVec &splits) = 0;
    virtual UStmVec unpool(bool beforeRewrite) = 0;
    virtual bool hasPool(bool beforeRewrite) const = 0;
    virtual bool check() const = 0;
    virtual void replace(Defines &dx) = 0;
    virtual void toGround(ToGroundArg &x, Ground::UStmVec &stms) const = 0;
    virtual ~Statement() { }
};

// }}}

} } // namespace Input Gringo

#endif // _GRINGO_INPUT_STATEMENT_HH
