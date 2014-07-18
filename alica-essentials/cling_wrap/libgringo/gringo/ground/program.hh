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

#ifndef _GRINGO_GROUND_PROGRAM_HH
#define _GRINGO_GROUND_PROGRAM_HH

#include <gringo/ground/statement.hh>

namespace Gringo { namespace Ground {

// {{{ declaration of Program

using IdVec       = std::vector<std::pair<Location, FWString>>;
using SEdb        = std::shared_ptr<std::pair<UTerm, ValVec>>;
using SEdbVec     = std::vector<SEdb>;
using FWValVecSet = std::set<FWValVec>;
using ParamSet    = std::map<FWSignature, FWValVecSet>;

struct Parameters {
    Parameters();
    void add(FWString name, FWValVec args);
    bool find(FWSignature sig) const;
    ParamSet::const_iterator begin() const;
    ParamSet::const_iterator end() const;
    bool empty() const;
    void clear();
    ~Parameters();
    ParamSet params;
};

struct Program {
    using ClassicalNegationVec = std::vector<std::tuple<PredicateDomain&, PredicateDomain&>>;

    Program(SEdbVec &&edb, Statement::Dep::ComponentVec &&stms, ClassicalNegationVec &&negate);
    void linearize(Scripts &scripts);
    void ground(Parameters const &params, Scripts &scripts, Output::OutputBase &out, bool finalize = true);
    void ground(Scripts &scripts, Output::OutputBase &out);
 
    SEdbVec                      edb;
    bool                         linearized = false;
    Statement::Dep::ComponentVec stms;
    ClassicalNegationVec         negate;
};

std::ostream &operator<<(std::ostream &out, Program const &x);

// }}}

} } // namespace Ground Gringo

#endif // _GRINGO_GROUND_PROGRAM_HH
