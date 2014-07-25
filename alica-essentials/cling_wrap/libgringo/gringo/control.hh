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

#ifndef _GRINGO_CONTROL_HH
#define _GRINGO_CONTROL_HH

#include <gringo/value.hh>

namespace Gringo {

enum class SolveResult { UNKNOWN=0, SAT=1, UNSAT=2 };

// {{{ declaration of Model

using Int64Vec = std::vector<int64_t>;

struct Model {
    static const unsigned CSP   = 1;
    static const unsigned SHOWN = 2;
    static const unsigned ATOMS = 4;
    static const unsigned TERMS = 8;
    virtual bool contains(Value atom) const = 0;
    virtual ValVec atoms(int showset) const = 0;
    virtual Int64Vec optimization() const = 0;
    virtual ~Model() { }
};

// }}}
// {{{ declaration of Statistics

struct Statistics {
    enum Error { error_none = 0, error_unknown_quantity = 1, error_ambiguous_quantity = 2, error_not_available = 3 };
    struct Quantity {
        Quantity(double d) : rep(d) { assert(d >= 0.0); }
        Quantity(Error e) : rep(-double(int(e))) { assert(e != error_none); }
        bool     valid()  const { return error() == error_none; }
        Error    error()  const { return rep >= 0.0 ? error_none : static_cast<Error>(int(-rep)); }
        operator bool()   const { return valid(); }
        operator double() const { return valid() ? rep : std::numeric_limits<double>::quiet_NaN(); }
    private:
        double rep;
    };
    virtual Quantity    getStat(char const* key) const = 0;
    virtual char const *getKeys(char const* key) const = 0;
    virtual ~Statistics() { }
};

// }}}
// {{{ declaration of SolveFuture

struct SolveFuture {
    virtual SolveResult get() = 0;
    virtual void wait() = 0;
    virtual bool wait(double timeout) = 0;
    virtual void interrupt() = 0;
    virtual ~SolveFuture() { }
};

// }}}
// {{{ declaration of Control

using FWStringVec = std::vector<FWString>;

struct Control {
    using ModelHandler = std::function<bool (Model const &)>;
    using FinishHandler = std::function<void (SolveResult, bool)>;
    virtual void ground(std::string const &name, FWValVec args) = 0;
    virtual SolveResult solve(ModelHandler h) = 0;
    virtual SolveFuture *asolve(ModelHandler mh, FinishHandler fh) = 0;
    virtual void add(std::string const &name, FWStringVec const &params, std::string const &part) = 0;
    virtual Value getConst(std::string const &name) = 0;
    virtual bool blocked() = 0;
    virtual void assignExternal(Value ext, bool val) = 0;
    virtual void releaseExternal(Value ext) = 0;
    virtual Statistics *getStats() = 0;
    virtual void setConf(std::string const &part, bool replace) = 0;
    virtual void enableEnumAssumption(bool enable) = 0;
    virtual ~Control() { }
};

// }}}

} // namespace Gringo

#endif // _GRINGO_CONTROL_HH

