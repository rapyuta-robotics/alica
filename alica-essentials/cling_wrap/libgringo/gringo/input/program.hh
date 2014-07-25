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

#ifndef _GRINGO_INPUT_PROGRAM_HH
#define _GRINGO_INPUT_PROGRAM_HH

#include <gringo/input/literal.hh>
#include <gringo/input/statement.hh>
#include <gringo/ground/program.hh>
#include <gringo/unique_list.hh>

namespace Gringo { namespace Input {

// {{{ declaration of Program

using IdVec = Ground::IdVec;

struct Block {
    Block(Location const &loc, FWString name, IdVec &&params);

    Term const &sig() const;
    operator Term const &() const;
    
    Location        loc;
    FWString        name;
    IdVec           params;
    ValVec          addedEdb;
    Ground::SEdb    edb;
    UStmVec         addedStms;
    UStmVec         stms;
};
using BlockMap = unique_list<Block, identity<Term>>;

class Program {
public:
    using ClassicalNegationList = unique_list<FWSignature>;

    Program();
    Program(Program &&x);
    void begin(Location const &loc, FWString name, IdVec &&params);
    void add(UStm &&stm);
    void addClassicalNegation(FWSignature x);
    void rewrite(Defines &defs);
    bool check();
    void print(std::ostream &out) const;
    Ground::Program toGround(PredDomMap &domains);
    ~Program();

private:
    void rewriteDots();
    void rewriteArithmetics();
    void unpool();

    unsigned              auxNames_ = 0;
    Ground::LocSet        locs_;
    Ground::SigSet        sigs_;
    BlockMap              blocks_;
    Block                *current_;
    Projections           project_;
    UStmVec               stms_;
    ClassicalNegationList neg_;
};

std::ostream &operator<<(std::ostream &out, Program const &p);

// }}}

} } // namespace Input Gringo

#endif //_GRINGO_INPUT_PROGRAM_HH
