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

#ifndef _GRINGO_CLONABLE_HH
#define _GRINGO_CLONABLE_HH

namespace Gringo {

// {{{ declaration of Clonable

template <class Base>
class Clonable {
public:
    virtual Base *clone() const = 0;
    virtual ~Clonable() { }
};

// }}}

} // namespace Gringo

#define GRINGO_CALL_CLONE(T) \
namespace Gringo { \
template <> \
struct clone<T> { \
    inline T operator()(T const &x) const { \
        return x.clone(); \
    } \
}; \
}

#endif // _GRINGO_CLONABLE_HH

