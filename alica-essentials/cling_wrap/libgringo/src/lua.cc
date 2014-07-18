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

#ifdef WITH_LUA

#include "gringo/lua.hh"
#include "gringo/logger.hh"
#include "gringo/control.hh"

#include <lua.hpp>
#include <cstring>

namespace Gringo {

namespace {

// {{{ auxiliary functions

#define VALUE_CMP(TYPE) \
int eq##TYPE(lua_State *L) { \
    Value *a = static_cast<Value*>(luaL_checkudata(L, 1, "gringo."#TYPE)); \
    Value *b = static_cast<Value*>(luaL_checkudata(L, 2, "gringo."#TYPE)); \
    lua_pushboolean(L, *a == *b); \
    return 1; \
} \
int lt##TYPE(lua_State *L) { \
    Value *a = static_cast<Value*>(luaL_checkudata(L, 1, "gringo."#TYPE)); \
    Value *b = static_cast<Value*>(luaL_checkudata(L, 2, "gringo."#TYPE)); \
    lua_pushboolean(L, *a <= *b); \
    return 1; \
} \
int le##TYPE(lua_State *L) { \
    Value *a = static_cast<Value*>(luaL_checkudata(L, 1, "gringo."#TYPE)); \
    Value *b = static_cast<Value*>(luaL_checkudata(L, 2, "gringo."#TYPE)); \
    lua_pushboolean(L, *a < *b); \
    return 1; \
}

template <typename T>
T *luaCppNew(lua_State *L, char const *meta) {
    T *ret = new ((ValVec*)lua_newuserdata(L, sizeof(T))) T();
    luaL_getmetatable(L, meta);
    lua_setmetatable(L, -2);
    return ret;
}

template <typename T>
int luaCppDelete(lua_State *L) {
    T* del = (T*)lua_touserdata(L, 1);
    del->T::~T();
    return 0;
}

template <typename R, typename T>
R protect(lua_State *L, T f) {
    try                             { return f(); }
    catch (std::exception const &e) { luaL_error(L, e.what()); }
    catch (...)                     { luaL_error(L, "unknown error"); }
    throw std::runtime_error("cannot happen");
}

Value luaToVal(lua_State *L, int idx) {
    int type = lua_type(L, idx);
    switch(type) {
        case LUA_TSTRING: {
            char const *name = lua_tostring(L, idx);
            return protect<Value>(L, [name]() { return Value(name, false); });
        }
        case LUA_TNUMBER: {
            int num = lua_tointeger(L, idx);
            return Value(num);
        }
        case LUA_TUSERDATA: {
            auto check = [L, idx]() -> bool {
                if (lua_getmetatable(L, idx)) {
                    lua_getfield(L, LUA_REGISTRYINDEX, "gringo.Fun");
                    if (lua_rawequal(L, -1, -2)) {
                        lua_pop(L, 2);
                        return true;
                    }
                    lua_pop(L, 1);
                    lua_getfield(L, LUA_REGISTRYINDEX, "gringo.Sup");
                    if (lua_rawequal(L, -1, -2)) {
                        lua_pop(L, 2);
                        return true;
                    }
                    lua_pop(L, 1);
                    lua_getfield(L, LUA_REGISTRYINDEX, "gringo.Inf");
                    if (lua_rawequal(L, -1, -2)) {
                        lua_pop(L, 2);
                        return true;
                    }
                    lua_pop(L, 1);
                }
                return false;
            };
            if (check()) { return *(Value*)lua_touserdata(L, idx); }
        }
        default: { luaL_error(L, "cannot convert to value"); }
    }
    throw std::runtime_error("cannot happen");
}
void valToLua(lua_State *L, Value v) {
    switch (v.type()) {
        case Value::ID:
        case Value::FUNC: {
            *(Value*)lua_newuserdata(L, sizeof(Value)) = v;
            luaL_getmetatable(L, "gringo.Fun");
            lua_setmetatable(L, -2);
            break;
        }
        case Value::SUP: {
            *(Value*)lua_newuserdata(L, sizeof(Value)) = v;
            luaL_getmetatable(L, "gringo.Sup");
            lua_setmetatable(L, -2);
            break;
        }
        case Value::INF: {
            *(Value*)lua_newuserdata(L, sizeof(Value)) = v;
            luaL_getmetatable(L, "gringo.Inf");
            lua_setmetatable(L, -2);
            break;
        }
        case Value::NUM: {
            lua_pushnumber(L, v.num());
            break;
        }
        case Value::STRING: {
            lua_pushstring(L, protect<char const *>(L, [v](){ return (*v.string()).c_str(); }));
            break;
        }
        default: { luaL_error(L, "cannot happen"); }
    }
}

ValVec *luaToVals(lua_State *L, int idx) {
    luaL_checktype(L, idx, LUA_TTABLE);
    ValVec *vals = luaCppNew<ValVec>(L, "gringo._ValVec");
    lua_pushnil(L);
    while (lua_next(L, idx) != 0) {
        Value val = luaToVal(L, -1);
        protect<void>(L, [val,&vals](){ vals->push_back(val); });
        lua_pop(L, 1);
    }
    return vals;
}

bool handleError(lua_State *L, Location const &loc, int code, char const *desc) {
    switch (code) {
        case LUA_ERRSYNTAX: {
            std::string msg(lua_tostring(L, -1));
            lua_pop(L, 1);
            GRINGO_REPORT(W_TERM_UNDEFINED)
                << loc << ": warning: " << desc << ":\n"
                << "  SyntaxError: "
                << msg << "\n"
                ;
            return false;
        }
        case LUA_ERRRUN:
        case LUA_ERRERR: {
            std::string msg(lua_tostring(L, -1));
            lua_pop(L, 1);
            GRINGO_REPORT(W_TERM_UNDEFINED)
                << loc << ": warning: " << desc << ":\n"
                << "  RuntimeError: "
                << msg << "\n"
                ;
            return false;
        }
        case LUA_ERRMEM: { throw std::runtime_error("lua interpreter ran out of memory"); }
    }
    return true;
}

static int luaTraceback (lua_State *L);

// }}}
// {{{ wrap Inf

int newInf(lua_State *L) {
    *(Value*)lua_newuserdata(L, sizeof(Value)) = Value(true);
    luaL_getmetatable(L, "gringo.Inf");
    lua_setmetatable(L, -2);
    return 1;
}
VALUE_CMP(Inf)
int toStringInf(lua_State *L) {
    luaL_checkudata(L, 1, "gringo.Inf");
    lua_pushliteral(L, "#inf");
    return 1;
}

// }}}
// {{{ wrap Sup

int newSup(lua_State *L) {
    *(Value*)lua_newuserdata(L, sizeof(Value)) = Value(false);
    luaL_getmetatable(L, "gringo.Sup");
    lua_setmetatable(L, -2);
    return 1;
}
VALUE_CMP(Sup)
int toStringSup(lua_State *L) {
    luaL_checkudata(L, 1, "gringo.Sup");
    lua_pushliteral(L, "#sup");
    return 1;
}

// }}}
// {{{ wrap Fun

int newFun(lua_State *L) {
    char const *name = luaL_checklstring(L, 1, nullptr);
    if (name[0] == '\0') { luaL_argerror(L, 2, "function symbols must have a non-empty name"); }
    if (lua_isnone(L, 2) || lua_isnil(L, 2)) {
        *(Value*)lua_newuserdata(L, sizeof(Value)) = protect<Value>(L, [name](){ return Value(name); });
        luaL_getmetatable(L, "gringo.Fun");
        lua_setmetatable(L, -2);
    }
    else {
        ValVec *vals = luaToVals(L, 2);
        *(Value*)lua_newuserdata(L, sizeof(Value)) = protect<Value>(L, [name, vals](){ return vals->empty() ? Value(name) : Value(name, *vals); });
        luaL_getmetatable(L, "gringo.Fun");
        lua_setmetatable(L, -2);
    }
    return 1;
}
int newTuple(lua_State *L) {
    ValVec *vals = luaToVals(L, 1);
    if (vals->size() < 2) { luaL_argerror(L, 1, "tuples must have at least two values"); }
    *(Value*)lua_newuserdata(L, sizeof(Value)) = protect<Value>(L, [vals](){ return Value("", *vals); });
    luaL_getmetatable(L, "gringo.Fun");
    lua_setmetatable(L, -2);
    return 1;
}
VALUE_CMP(Fun)
int nameFun(lua_State *L) {
    Value val = *(Value*)luaL_checkudata(L, 1, "gringo.Fun");
    lua_pushstring(L, protect<const char*>(L, [val]() { return (*val.name()).c_str(); }));
    return 1;
}
int argsFun(lua_State *L) {
    Value val = *(Value*)luaL_checkudata(L, 1, "gringo.Fun");
    lua_createtable(L, val.args().size(), 0);
    if (val.type() == Value::FUNC) {
        int i = 1;
        for (auto &x : val.args()) {
            valToLua(L, x);
            lua_rawseti(L, -2, i++);
        }
    }
    return 1;
}
int toStringFun(lua_State *L) {
    std::string *rep = luaCppNew<std::string>(L, "gringo._string");
    Value val = *(Value*)luaL_checkudata(L, 1, "gringo.Fun");
    lua_pushstring(L, protect<const char*>(L, [val, rep]() {
        std::ostringstream oss;
        oss << val;
        *rep = oss.str();
        return rep->c_str();
    }));
    return 1;
}

// }}}
// {{{ wrap Model

int containsModel(lua_State *L) { 
    Model const *& model =  *(Model const **)luaL_checkudata(L, 1, "gringo.Model");
    Value val = luaToVal(L, 2);
    lua_pushboolean(L, protect<bool>(L, [val, model]() { return model->contains(val); }));
    return 1;
}
int atomsModel(lua_State *L) {
    Model const *& model = *(Model const **)luaL_checkudata(L, 1, "gringo.Model");
    int atomset = Gringo::Model::SHOWN;
    if (lua_isnumber (L, 2)) { atomset = luaL_checkinteger(L, 2); }
    ValVec *atoms = luaCppNew<ValVec>(L, "gringo._ValVec");
    protect<void>(L, [&model, atoms, atomset]() { *atoms = model->atoms(atomset); });
    lua_createtable(L, atoms->size(), 0);
    int i = 1;
    for (auto x : *atoms) { 
        valToLua(L, x);
        lua_rawseti(L, -2, i++);
    }
    return 1;
}
int optimizationModel(lua_State *L) {
    Model const *& model = *(Model const **)luaL_checkudata(L, 1, "gringo.Model");
    Int64Vec *values = luaCppNew<Int64Vec>(L, "gringo._Int64Vec");
    protect<void>(L, [&model, values]() { *values = model->optimization(); });
    lua_createtable(L, values->size(), 0);
    int i = 1;
    for (auto x : *values) { 
        lua_pushinteger(L, x);
        lua_rawseti(L, -2, i++);
    }
    return 1;
}
int toStringModel(lua_State *L) { 
    Model const *& model =  *(Model const **)luaL_checkudata(L, 1, "gringo.Model");
    std::string *rep = luaCppNew<std::string>(L, "gringo._string");
    lua_pushstring(L, protect<char const *>(L, [model, rep]() { 
        auto printAtom = [](std::ostream &out, Value val) {
            if (val.type() == Value::FUNC && *val.sig() == Signature("$", 2)) { out << val.args().front() << "=" << val.args().back(); }
            else { out << val; }
        };
        std::ostringstream oss;
        print_comma(oss, model->atoms(Model::SHOWN), " ", printAtom);
        *rep = oss.str();
        return rep->c_str();
    }));
    return 1;
}

// }}}
// {{{ wrap Statistics

int newStatistics(lua_State *L, Statistics const *stats, int idxPrefix) {
    char const *prefix = lua_tostring(L, idxPrefix);
    auto ret           = protect<Statistics::Quantity>(L, [stats, prefix]{ return stats->getStat(prefix); });
    switch (ret.error()) {
        case Statistics::error_none: { 
            lua_pushnumber(L, (double)ret); 
            break;
        }
        case Statistics::error_not_available: {
            luaL_error(L, "error_not_available: %s", prefix);
            break;
        }
        case Statistics::error_unknown_quantity: { 
            luaL_error(L, "error_unknown_quantity: %s", prefix);
            break;
        }
        case Statistics::error_ambiguous_quantity: {
            lua_newtable(L);
            char const *keys = protect<char const *>(L, [stats, prefix]() { return stats->getKeys(prefix); });
            if (!keys) { luaL_error(L, "error zero keys string: %s", prefix); }
            for (char const *it = keys; *it; it+= strlen(it) + 1) {
                lua_pushvalue(L, idxPrefix);
                lua_pushstring(L, it);
                lua_concat(L, 2);
                if (strcmp(it, "__len") == 0) {
                    char const *lenPrefix = lua_tostring(L, -1);
                    int len = (int)protect<double>(L, [stats, lenPrefix]{ return stats->getStat(lenPrefix); });
                    lua_pop(L, 1);
                    for (int i = 1; i <= len; ++i) {
                        lua_pushvalue(L, idxPrefix);
                        lua_pushinteger(L, i-1);
                        lua_pushliteral(L, ".");
                        lua_concat(L, 3);
                        newStatistics(L, stats, lua_gettop(L));
                        lua_rawseti(L, -3, i);
                        lua_pop(L, 1);
                    }
                    break;
                }
                else {
                    int len = strlen(it);
                    lua_pushlstring(L, it, len - (it[len-1] == '.'));
                    newStatistics(L, stats, lua_gettop(L) - 1);
                    lua_rawset(L, -4);
                    lua_pop(L, 1);
                }
            }
            break;
        }
        
    }
    return 1;
}

// }}}
// {{{ wrap SolveFuture

int getSolveFuture(lua_State *L) { 
    SolveFuture *& future = *(SolveFuture **)luaL_checkudata(L, 1, "gringo.SolveFuture");
    lua_pushnumber(L, protect<int>(L, [future]() { return (int)future->get(); }));
    return 1;
}
int waitSolveFuture(lua_State *L) {
    SolveFuture *& future = *(SolveFuture **)luaL_checkudata(L, 1, "gringo.SolveFuture");
    if (lua_isnone(L, 2) == 0) {
        double timeout = luaL_checknumber(L, 2);
        lua_pushboolean(L, protect<bool>(L, [future, timeout]() { return future->wait(timeout); }));
        return 1;
    }
    else {
        protect<void>(L, [future]() { future->wait(); });
        return 0;
    }
}
int interruptSolveFuture(lua_State *L) { 
    SolveFuture *& future = *(SolveFuture **)luaL_checkudata(L, 1, "gringo.SolveFuture");
    protect<void>(L, [future]() { future->interrupt(); });
    return 0;
}

// }}}
// {{{ wrap Control

struct LuaClear {
    LuaClear(lua_State *L) : L(L), n(lua_gettop(L)) { }
    ~LuaClear() { lua_settop(L, n); }
    lua_State *L;
    int n;
};
void checkBlocked(lua_State *L, Control *ctl, char const *function) {
    if (protect<bool>(L, [ctl]() { return ctl->blocked(); })) { luaL_error(L, "Control.%s must not be called during solve call", function); }
}
int groundControl(lua_State *L) {
    auto &ctl =  *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "ground");
    char const *name = luaL_checkstring(L, 2);
    ValVec *args = luaToVals(L, 3);
    protect<void>(L, [ctl, name, args]() { ctl->ground(name, *args); });
    return 0;
}
int addControl(lua_State *L) { 
    auto &ctl = *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "add");
    char const *name = luaL_checkstring(L, 2);
    luaL_checktype(L, 3, LUA_TTABLE);
    char const *prg = luaL_checkstring(L, 4);
    FWStringVec *vals = luaCppNew<FWStringVec>(L, "gringo._FWStringVec");
    lua_pushnil(L);
    while (lua_next(L, 3) != 0) {
        char const *val = luaL_checkstring(L, -1);
        protect<void>(L, [val,&vals](){ vals->push_back(val); });
        lua_pop(L, 1);
    }
    protect<void>(L, [ctl, name, vals, prg]() { ctl->add(name, *vals, prg); });
    return 0;
}
int getConstControl(lua_State *L) { 
    auto &ctl = *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "getConst");
    char const *name = luaL_checkstring(L, 2);
    Value ret = protect<Value>(L, [ctl, name]() { return ctl->getConst(name); });
    if (ret.type() == Value::SPECIAL) { lua_pushnil(L); }
    else                              { valToLua(L, ret); }
    return 1;
}
int solveControl(lua_State *L) {
    auto &ctl = *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "solve");
    Model const **model = nullptr;
    if (lua_isnone(L, 2) == 0 && lua_isnil(L, 2) == 0) {
        model = (Model const **)lua_newuserdata(L, sizeof(Model*));
        luaL_getmetatable(L, "gringo.Model");
        lua_setmetatable(L, -2);
    }
    lua_pushinteger(L, protect<int>(L, [L, ctl, model](){ 
        return (int)ctl->solve(!model ? Control::ModelHandler(nullptr) : [L, model](Model const &m) -> bool {
            LuaClear lc(L);
            lua_pushcfunction(L, luaTraceback);
            lua_pushvalue(L, 2);
            lua_pushvalue(L, -3);
            *model = &m;
            int code = lua_pcall(L, 1, 1, -3);
            Location loc("<onModel>", 1, 1, "<onModel>", 1, 1);
            if (!handleError(L, loc, code, "error in model callback")) { throw std::runtime_error("error in model callback"); }
            return lua_type(L, -1) == LUA_TNIL || lua_toboolean(L, -1);
        });
    }));
    return 1;
}
int asolveControl(lua_State *L) {
    auto &ctl = *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "asolve");
    int mhIndex = !lua_isnone(L, 2) && !lua_isnil(L, 2);
    int fhIndex = !lua_isnone(L, 3) && !lua_isnil(L, 3);
    auto &future = *(SolveFuture **)lua_newuserdata(L, sizeof(Model*));
    lua_State *M = nullptr;
    if (mhIndex || fhIndex) {
        lua_getfield(L, LUA_REGISTRYINDEX, "gringo._SolveThread");
        M = (lua_State*)lua_tothread(L, -1);
        lua_pop(L, 1);
        lua_settop(M, 0);
    }
    Model const ** model = nullptr;
    if (mhIndex) {
        lua_pushvalue(L, 2);
        lua_xmove(L, M, 1);
        mhIndex = lua_gettop(M);
        model   = (Model const **)lua_newuserdata(M, sizeof(Model*));
        luaL_getmetatable(M, "gringo.Model");
        lua_setmetatable(M, -2);
    }
    if (fhIndex) {
        lua_pushvalue(L, 3);
        lua_xmove(L, M, 1);
        fhIndex = lua_gettop(M);
    }
    future = protect<SolveFuture*>(L, [ctl, model, mhIndex, fhIndex, M]() { 
        auto mh = !mhIndex ? Control::ModelHandler(nullptr) : [M, mhIndex, model](Model const &m) -> bool {
            LuaClear lc(M);
            lua_pushcfunction(M, luaTraceback);
            lua_pushvalue(M, mhIndex);
            lua_pushvalue(M, mhIndex+1);
            *model = &m;
            int code = lua_pcall(M, 1, 1, -3);
            Location loc("<onModel>", 1, 1, "<onModel>", 1, 1);
            if (!handleError(M, loc, code, "error in model callback")) { throw std::runtime_error("error in model callback"); }
            return lua_type(M, -1) == LUA_TNIL || lua_toboolean(M, -1);
        };
        auto fh = !fhIndex ? Control::FinishHandler(nullptr) : [M, fhIndex](SolveResult ret, bool interrupted) -> void {
            LuaClear lc(M);
            lua_pushcfunction(M, luaTraceback);
            lua_pushvalue(M, fhIndex);
            lua_pushinteger(M, (int)ret);
            lua_pushboolean(M, interrupted);
            int code = lua_pcall(M, 2, 1, -4);
            Location loc("<onModel>", 1, 1, "<onModel>", 1, 1);
            if (!handleError(M, loc, code, "error in model callback")) { throw std::runtime_error("error in model callback"); }
        };
        return ctl->asolve(mh, fh);
    });
    luaL_getmetatable(L, "gringo.SolveFuture");
    lua_setmetatable(L, -2);
    return 1;
}
int assignExternalControl(lua_State *L) { 
    auto &ctl = *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "assignExternal");
    Value atom = luaToVal(L, 2);
    luaL_checktype(L, 3, LUA_TBOOLEAN);
    bool truth = lua_toboolean(L, 3);
    protect<void>(L, [ctl, atom, truth]() { ctl->assignExternal(atom, truth); });
    return 0;
}
int releaseExternalControl(lua_State *L) { 
    auto &ctl = *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "releaseExternal");
    Value atom = luaToVal(L, 2);
    protect<void>(L, [ctl, atom]() { ctl->releaseExternal(atom); });
    return 0;
}
int getStatsControl(lua_State *L) {
    auto &ctl = *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "getStats");
    auto stats = protect<Statistics *const>(L, [ctl](){ return ctl->getStats(); });
    lua_pushliteral(L, "");
    return newStatistics(L, stats, lua_gettop(L));
}
int setConfControl(lua_State *L) {
    auto &ctl = *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "setConf");
    char const *config = luaL_checkstring(L, 2);
    bool replace = false;
    if (!lua_isnone(L, 3)) {
        if (!lua_isboolean(L, 3)) { return luaL_argerror(L, 3, "boolean expected"); }
        replace = lua_toboolean(L, 3);
    }
    protect<void>(L, [ctl, config, replace]() { ctl->setConf(config, replace); });
    return 0;
}
int enableEnumAssumptionControl(lua_State *L) {
    auto &ctl = *(Control **)luaL_checkudata(L, 1, "gringo.Control");
    checkBlocked(L, ctl, "enableEnumAssumption");
    luaL_checktype(L, 2, LUA_TBOOLEAN);
    bool enable = lua_toboolean(L, 2);
    protect<void>(L, [ctl, enable]() { ctl->enableEnumAssumption(enable); });
    return 0;
}

// }}}
// {{{ wrap cmp

int cmpVal(lua_State *L) {
    Value a(luaToVal(L, 1));
    Value b(luaToVal(L, 2));
    if (a < b)      { lua_pushnumber(L, -1); }
    else if (b < a) { lua_pushnumber(L, 1); }
    else            { lua_pushnumber(L, 0); }
    return 1;
}

// }}}
// {{{ gringo library

int luaopen_gringo(lua_State* L) {
    static luaL_Reg const gringoLib[] = {
        {"Inf",      newInf},
        {"Sup",      newSup},
        {"Fun",      newFun},
        {"Tuple",    newTuple},
        {"cmp",      cmpVal},
        {nullptr,    nullptr}
    };
    static luaL_Reg const infMeta[] = {
        {"__tostring", toStringInf},
        {"__eq", eqInf},
        {"__lt", ltInf},
        {"__le", leInf},
        {nullptr, nullptr}
    };
    static luaL_Reg const supMeta[] = {
        {"__tostring", toStringSup},
        {"__eq", eqSup},
        {"__lt", ltSup},
        {"__le", leSup},
        {nullptr, nullptr}
    };
    static luaL_Reg const funMeta[] = {
        {"__tostring", toStringFun},
        {"name", nameFun},
        {"args", argsFun},
        {"__eq", eqFun},
        {"__lt", ltFun},
        {"__le", leFun},
        {nullptr, nullptr}
    };
    static luaL_Reg const modelMeta[] = {
        {"__tostring", toStringModel},
        {"atoms", atomsModel},
        {"optimization", optimizationModel},
        {"contains", containsModel},
        {nullptr, nullptr}
    };
    static luaL_Reg const solveFutureMeta[] = {
        {"get",  getSolveFuture},
        {"wait", waitSolveFuture},
        {"interrupt", interruptSolveFuture},
        {nullptr, nullptr}
    };
    static luaL_Reg const controlMeta[] = {
        {"ground",  groundControl},
        {"add", addControl},
        {"solve", solveControl},
        {"asolve", asolveControl},
        {"getConst", getConstControl},
        {"assignExternal", assignExternalControl},
        {"releaseExternal", releaseExternalControl},
        {"getStats", getStatsControl},
        {"setConf", setConfControl},
        {"enableEnumAssumption", enableEnumAssumptionControl},
        {nullptr, nullptr}
    };
    static luaL_Reg const _stringMeta[] = {
        {"__gc", luaCppDelete<std::string>},
        {nullptr, nullptr}
    };
    static luaL_Reg const _ValVecMeta[] = {
        {"__gc", luaCppDelete<ValVec>},
        {nullptr, nullptr}
    };
    static luaL_Reg const _Int64VecMeta[] = {
        {"__gc", luaCppDelete<Int64Vec>},
        {nullptr, nullptr}
    };
    static luaL_Reg const _FWStringVecMeta[] = {
        {"__gc", luaCppDelete<FWStringVec>},
        {nullptr, nullptr}
    };
    // Note: in gringo-3 I also set the metatable attribute
    auto regMeta = [L](char const *name, luaL_Reg const * funs) {
#if LUA_VERSION_NUM < 502
        luaL_newmetatable(L, name);    // push metatable inf
        luaL_register(L, 0, funs);     // push meta
#else
        luaL_newmetatable(L, name);    // push metatable inf
        luaL_setfuncs(L, funs, 0);
#endif
        lua_pushliteral(L, "__metatable"); // push __metatable
        lua_pushvalue(L, -2);              // push inf
        lua_rawset(L, -3);                 // inf._metatablex = inf
        lua_pushliteral(L, "__index");     // push __index
        lua_pushvalue(L, -2);              // push inf
        lua_rawset(L, -3);                 // inf.__index = inf
    };

    lua_newthread(L);
    lua_setfield(L, LUA_REGISTRYINDEX, "gringo._SolveThread");

    regMeta("gringo.Inf",          infMeta);
    regMeta("gringo.Sup",          supMeta);
    regMeta("gringo.Fun",          funMeta);
    regMeta("gringo.Model",        modelMeta);
    regMeta("gringo.SolveFuture",  solveFutureMeta);
    regMeta("gringo.Control",      controlMeta);
    regMeta("gringo._string",      _stringMeta);
    regMeta("gringo._ValVec",      _ValVecMeta);
    regMeta("gringo._Int64Vec",    _Int64VecMeta);
    regMeta("gringo._FWStringVec", _FWStringVecMeta);

#if LUA_VERSION_NUM < 502
    luaL_register(L, "gringo", gringoLib);
#else
    luaL_newlib(L, gringoLib);
#endif

    lua_createtable(L, 0, 3);
    lua_pushinteger(L, (int)Gringo::SolveResult::SAT);
    lua_setfield(L, -2, "SAT");
    lua_pushinteger(L, (int)Gringo::SolveResult::UNSAT);
    lua_setfield(L, -2, "UNSAT");
    lua_pushinteger(L, (int)Gringo::SolveResult::UNKNOWN);
    lua_setfield(L, -2, "UNKNOWN");
    lua_setfield(L, -2, "SolveResult");

    lua_createtable(L, 0, 4);
    lua_pushinteger(L, (int)Gringo::Model::ATOMS);
    lua_setfield(L, -2, "ATOMS");
    lua_pushinteger(L, (int)Gringo::Model::TERMS);
    lua_setfield(L, -2, "TERMS");
    lua_pushinteger(L, (int)Gringo::Model::SHOWN);
    lua_setfield(L, -2, "SHOWN");
    lua_pushinteger(L, (int)Gringo::Model::CSP);
    lua_setfield(L, -2, "CSP");
    lua_setfield(L, -2, "Model");

    return 1;
}

int luarequire_gringo(lua_State *L) {
    luaL_openlibs(L);
#if LUA_VERSION_NUM < 502
    lua_pushcfunction(L, luaopen_gringo);
    lua_call(L, 0, 1);
#else
    luaL_requiref(L, "gringo", luaopen_gringo, true);
#endif
    return 0;
}

// }}}
// {{{ lua C functions

using LuaCallArgs = std::tuple<char const *, ValVec const &, ValVec>;

static int luaTraceback (lua_State *L) {
    if (!lua_isstring(L, 1)) { return 1; }
    lua_getglobal(L, "debug");
    if (!lua_istable(L, -1)) {
        lua_pop(L, 1);
        return 1;
    }
    lua_getfield(L, -1, "traceback");
    if (!lua_isfunction(L, -1)) {
        lua_pop(L, 2);
        return 1;
    }
    lua_pushvalue(L, 1);
    lua_pushinteger(L, 2);
    lua_call(L, 2, 1);
    lua_getglobal(L, "string");
    if (!lua_istable(L, -1)) {
        lua_pop(L, 1);
        return 1;
    }
    lua_getfield(L, -1, "gsub");
    if (!lua_isfunction(L, -1)) {
        lua_pop(L, 1);
        return 1;
    }
    lua_pushvalue(L, -3);
    lua_pushliteral(L, "\t");
    lua_pushliteral(L, "  ");
    lua_call(L, 3, 1);
    return 1;
}

int luaCall(lua_State *L) {
    auto &args = *(LuaCallArgs*)lua_touserdata(L, 1);
    lua_getglobal(L, std::get<0>(args));
    for (auto &x : std::get<1>(args)) { valToLua(L, x); }
    lua_call(L, std::get<1>(args).size(), 1);
    if (lua_type(L, -1) == LUA_TTABLE) {
        lua_pushnil(L);
        while (lua_next(L, -2) != 0) {
            Value val = luaToVal(L, -1);
            protect<void>(L, [val, &args]() { std::get<2>(args).emplace_back(val); });
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
    }
    else {
        Value val = luaToVal(L, -1);
        protect<void>(L, [val, &args]() { std::get<2>(args).emplace_back(val); });
    }
    return 0;
}

int luaMain(lua_State *L) {
    auto ctl = (Control*)lua_touserdata(L, 1);
    lua_getglobal(L, "main");
    *(Control **)lua_newuserdata(L, sizeof(Control*)) = ctl;
    luaL_getmetatable(L, "gringo.Control");
    lua_setmetatable(L, -2);
    lua_call(L, 1, 0);
    return 0;
}

// }}}

} // namespace

// {{{ definition of LuaImpl

struct LuaImpl {
    LuaImpl() : L(luaL_newstate()) {
        if (!L) { throw std::runtime_error("could not open lua state"); }
        int n = lua_gettop(L);
        lua_pushcfunction(L, luaTraceback);
        lua_pushcfunction(L, luarequire_gringo);
        int ret = lua_pcall(L, 0, 0, -2);
        Location loc("<LuaImpl>", 1, 1, "<LuaImpl>", 1, 1);
        if (!handleError(L, loc, ret, "running lua script failed")) {
            throw std::runtime_error("could not initialize gringo module");
        }
        lua_settop(L, n);
    }
    ~LuaImpl() { 
        if (L) { lua_close(L); }
    }
    lua_State *L;
};

// }}}
// {{{ definition of Lua

Lua::Lua() = default;

bool Lua::exec(Location const &loc, FWString code) {
    if (!impl) { impl = make_unique<LuaImpl>(); }
    LuaClear lc(impl->L);
    std::stringstream oss;
    oss << loc;
    lua_pushcfunction(impl->L, luaTraceback);
    int ret = luaL_loadbuffer(impl->L, (*code).c_str(), (*code).size(), oss.str().c_str());
    if (!handleError(impl->L, loc, ret, "parsing lua script failed")) { return false; }
    ret = lua_pcall(impl->L, 0, 0, -2);
    if (!handleError(impl->L, loc, ret, "running lua script failed")) { return false; }
    return true;
}

ValVec Lua::call(Location const &loc, FWString name, ValVec const &args) {
    assert(impl);
    LuaClear lc(impl->L);
    LuaCallArgs arg((*name).c_str(), args, {});
    lua_pushcfunction(impl->L, luaTraceback);
    lua_pushcfunction(impl->L, luaCall);
    lua_pushlightuserdata(impl->L, (void*)&arg);
    int ret = lua_pcall(impl->L, 1, 0, -3);
    if (!handleError(impl->L, loc, ret, "operation undefined, a zero is substituted")) { return {0}; }
    return std::move(std::get<2>(arg));
}

bool Lua::callable(FWString name) {
    if (!impl) { return false; }
    LuaClear lc(impl->L);
    lua_getglobal(impl->L, (*name).c_str());
    bool ret = lua_type(impl->L, -1) == LUA_TFUNCTION;
    return ret;
}

void Lua::main(Control &ctl) {
    assert(impl);
    LuaClear lc(impl->L);
    lua_pushcfunction(impl->L, luaTraceback);
    lua_pushcfunction(impl->L, luaMain);
    lua_pushlightuserdata(impl->L, (void*)&ctl);
    switch (lua_pcall(impl->L, 1, 0, -3)) {
        case LUA_ERRRUN:
        case LUA_ERRERR: {
            std::ostringstream oss;
            oss << lua_tostring(impl->L, -1);
            lua_pop(impl->L, 1);
            throw std::runtime_error(oss.str());
        }
        case LUA_ERRMEM: { throw std::runtime_error("lua interpreter ran out of memory"); }
    }
}
Lua::~Lua() = default;

// }}}

} // namespace Gringo

#else // WITH_LUA

#include "gringo/lua.hh"
#include "gringo/logger.hh"

namespace Gringo {

// {{{ definition of LuaImpl

struct LuaImpl { };

// }}}
// {{{ definition of Lua

Lua::Lua() = default;
bool Lua::exec(Location const &loc, FWString) {
    GRINGO_REPORT(W_TERM_UNDEFINED)
        << loc << ": warning: gringo has been build without lua support, code is ignored\n"
        ;
    return false;
}
bool Lua::callable(FWString) {
    return false;
}
ValVec Lua::call(Location const &, FWString, ValVec const &) {
    return {0};
}
void Lua::main(Control &) { }
Lua::~Lua() = default;

// }}}

} // namespace Gringo

#endif // WITH_LUA

