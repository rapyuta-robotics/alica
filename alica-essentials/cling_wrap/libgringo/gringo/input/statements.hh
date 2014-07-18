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

#ifndef _GRINGO_INPUT_STATEMENTS_HH
#define _GRINGO_INPUT_STATEMENTS_HH

#include <gringo/input/statement.hh>
#include <gringo/input/aggregate.hh>

namespace Gringo { namespace Input {

// {{{ declaration of Rule

struct Rule : Statement {
    Rule(UHeadAggr &&head, UBodyAggrVec &&body, bool external);
    virtual UStmVec unpool(bool beforeRewrite);
    virtual void rewrite1(Projections &project);
    virtual void rewrite2(SplitVec &splits);
    virtual Value isEDB() const;
    virtual void print(std::ostream &out) const;
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check() const;
    virtual void replace(Defines &dx);
    virtual void toGround(ToGroundArg &x, Ground::UStmVec &stms) const;
    virtual void add(ULit &&lit);
    virtual ~Rule();

    UHeadAggr    head;
    UBodyAggrVec body;
    bool         external;
};

// }}}
// {{{ declaration of WeakConstraint

struct WeakConstraint : Statement {
    WeakConstraint(UTerm &&weight, UTerm &&prioriy, UTermVec &&tuple, UBodyAggrVec &&body);
    WeakConstraint(UTermVec &&tuple, UBodyAggrVec &&body);
    virtual UStmVec unpool(bool beforeRewrite);
    virtual void rewrite1(Projections &project);
    virtual void rewrite2(SplitVec &splits);
    virtual void print(std::ostream &out) const;
    virtual bool hasPool(bool beforeRewrite) const;
    virtual bool check() const;
    virtual void replace(Defines &dx);
    virtual void toGround(ToGroundArg &x, Ground::UStmVec &stms) const;
    virtual void add(ULit &&lit);
    virtual ~WeakConstraint();

    UTermVec     tuple;
    UBodyAggrVec body;
};

// }}}

} } // namespace Input Gringo

#endif // _GRINGO_INPUT_STATEMENTS_HH

