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

#ifndef _GRINGO_SCRIPTS_HH
#define _GRINGO_SCRIPTS_HH

#include <gringo/python.hh>
#include <gringo/lua.hh>

namespace Gringo {

struct Scripts {
    Scripts();
    bool pyExec(Location const &loc, FWString code);
    bool luaExec(Location const &loc, FWString code);
    bool callable(FWString name);
    ValVec call(Location const &loc, FWString name, ValVec const &args);
    void main(Control &ctl);
    ~Scripts();

    Python py;
    Lua lua;
};

} // namespace Gringo

#endif // _GRINGO_SCRIPTS_HH

