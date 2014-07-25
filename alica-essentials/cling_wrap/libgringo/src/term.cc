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

#include "gringo/term.hh"
#include "gringo/logger.hh"
#include "gringo/graph.hh"

namespace Gringo {

// {{{ definition of Defines

Defines::Defines() = default;
Defines::Defines(Defines &&) = default;

Defines::DefMap const &Defines::defs() const {
    return defs_;
}

void Defines::add(Location const &loc, FWString name, UTerm &&value, bool defaultDef) {
    auto it = defs_.find(name);
    if (it == defs_.end())                           { defs_.emplace(name, make_tuple(defaultDef, loc, std::move(value))); }
    else if (std::get<0>(it->second) && !defaultDef) { it->second = make_tuple(defaultDef, loc, std::move(value)); }
    else if (std::get<0>(it->second) || !defaultDef) {
        GRINGO_REPORT(W_DEFINE_REDEFINTION) 
            << loc << ": warning: redefinition of constant ignored:\n"
            << "  #const " << *name << "=" << *value << ".\n"
            << std::get<1>(it->second) << ": note: constant first defined here\n";
    }
}

void Defines::init() {
    using DefineGraph = Graph<Defines::DefMap::iterator>;
    using NodeMap     = std::unordered_map<FWString, DefineGraph::Node*>;

    DefineGraph graph;
    NodeMap nodes;
    for (auto it = defs_.begin(), end = defs_.end(); it != end; ++it) {
        nodes.emplace(it->first, &graph.insertNode(it));
    }
    for (auto &x : nodes) {
        Term::VarSet vals;
        std::get<2>(x.second->data->second)->collectIds(vals);
        for (auto &y : vals) {
            auto it = nodes.find(y);
            if (it != nodes.end()) { x.second->insertEdge(*it->second); }
        }
    }
    for (auto &scc : graph.tarjan()) {
        if (scc.size() > 1) {
            std::ostringstream msg;
            msg
                << std::get<1>(scc.back()->data->second) << ": warning: cyclic constant definition:\n"
                << "  #const " << *scc.back()->data->first << "=" << *std::get<2>(scc.back()->data->second) << ".\n";
            scc.pop_back();
            for (auto &x : scc) {
                msg
                    << std::get<1>(x->data->second) << ": note: cycle involves definition:\n"
                    << "  #const " << *x->data->first << "=" << *std::get<2>(x->data->second) << ".\n";
            }
            GRINGO_REPORT(W_DEFINE_CYCLIC) << msg.str();
        }
        for (auto &x : scc) { Term::replace(std::get<2>(x->data->second), std::get<2>(x->data->second)->replace(*this, true)); }
    }
}

bool Defines::empty() const { return defs_.empty(); }

void Defines::apply(Value x, Value &retVal, UTerm &retTerm, bool replace) {
    switch (x.type())  {
        case Value::FUNC: {
            ValVec args;
            bool changed = true;
            for (auto it = x.args().begin(), ie = x.args().end(); it != ie; ++it) {
                UTerm rt;
                args.emplace_back();
                apply(*it, args.back(), rt, true);
                if (rt) {
                    Location loc{rt->loc()};
                    UTermVec tArgs;
                    args.pop_back();
                    for (auto &y : args) { tArgs.emplace_back(make_locatable<ValTerm>(rt->loc(), y)); }
                    tArgs.emplace_back(std::move(rt));
                    for (++it; it != ie; ++it) {
                        Value rv;
                        tArgs.emplace_back();
                        apply(*it, rv, tArgs.back(), true);
                        if (!tArgs.back()) {
                            if (rv.type() == Value::SPECIAL) { rv = *it; }
                            tArgs.back() = make_locatable<ValTerm>(loc, rv);
                        }
                    }
                    retTerm = make_locatable<FunctionTerm>(loc, x.name(), std::move(tArgs));
                    return;
                }
                else if (args.back().type() == Value::SPECIAL) { args.back() = *it; }
                else                                           { changed = true; }
            }
            if (changed) { retVal = Value(x.name(), args); }
            break;
        }
        case Value::ID: {
            if (replace) {
                auto it(defs_.find(x.string()));
                if (it != defs_.end()) {
                    retVal = std::get<2>(it->second)->isEDB();
                    if (retVal.type() == Value::SPECIAL) {
                        retTerm = get_clone(std::get<2>(it->second));
                    }
                }
            }
            break;
        }
        default: { break; }
    }
}

Defines::~Defines() { }

// }}}

// {{{ definition of GRef

GRef::GRef(UTerm &&name)
    : type(EMPTY)
    , name(std::move(name))
    , value(0)
    , term(0) { }

GRef::operator bool() const { return type != EMPTY; }

void GRef::reset() { type = EMPTY; }

GRef &GRef::operator=(Value const &x) {
    type  = VALUE;
    value = x;
    return *this;
}

GRef &GRef::operator=(GTerm &x) {
    type = TERM;
    term = &x;
    return *this;
}

bool GRef::occurs(GRef &x) const {
    switch (type) {
        case VALUE: { return false; }
        case TERM:  { return term->occurs(x); }
        case EMPTY: { return this == &x; }
    }
    assert(false);
    return false;
}

bool GRef::match(Value const &x) {
    switch (type) {
        case VALUE: { return value == x; }
        case TERM:  { return term->match(x); }
        case EMPTY: { assert(false); }
    }
    return false;
}

template <class T>
bool GRef::unify(T &x) {
switch (type) {
    case VALUE: { return x.match(value); }
    case TERM:  { return term->unify(x); }
    case EMPTY: { assert(false); }
}
return false;

}

// }}}
// {{{ definition of G*Term::G*Term

GValTerm::GValTerm(Value val) : val(val) { }
GFunctionTerm::GFunctionTerm(FWString name, UGTermVec &&args) : name(name), args(std::move(args)) { } 
GLinearTerm::GLinearTerm(SGRef ref, int m, int n) : ref(ref), m(m), n(n) { assert(ref); }
GVarTerm::GVarTerm(SGRef ref) : ref(ref) { assert(ref); }

// }}}
// {{{ definition of G*Term::operator==

// Note: uses structural comparosson of names of variabl terms (VarTerm/LinearTerm)
bool GValTerm::operator==(GTerm const &x) const {
auto t = dynamic_cast<GValTerm const*>(&x);
return t && val == t->val;
}
bool GFunctionTerm::operator==(GTerm const &x) const {
auto t = dynamic_cast<GFunctionTerm const*>(&x);
return t && name == t->name && is_value_equal_to(args, t->args);
}
bool GLinearTerm::operator==(GTerm const &x) const {
auto t = dynamic_cast<GLinearTerm const*>(&x);
return t && *ref->name == *t->ref->name && m == t->m && n == t->n;
}
bool GVarTerm::operator==(GTerm const &x) const {
auto t = dynamic_cast<GVarTerm const*>(&x);
return t && *ref->name == *t->ref->name;
}

// }}}
// {{{ definition of G*Term::hash

size_t GValTerm::hash() const      { return get_value_hash(typeid(GValTerm).hash_code(), val); }
size_t GFunctionTerm::hash() const { return get_value_hash(typeid(GFunctionTerm).hash_code(), name, args); }
size_t GLinearTerm::hash() const   { return get_value_hash(typeid(GLinearTerm).hash_code(), ref->name, m, n); }
size_t GVarTerm::hash() const      { return get_value_hash(typeid(GVarTerm).hash_code(), ref->name); }

// }}}
// {{{ definition of G*Term::print

void GValTerm::print(std::ostream &out) const { out << val; }
void GFunctionTerm::print(std::ostream &out) const {
    out << name;
    out << "(";
    print_comma(out, args, ",", [](std::ostream &out, UGTerm const &x) { out << *x; });
    out << ")";
}
void GLinearTerm::print(std::ostream &out) const { out << "(" << m << "*" << *ref->name << "+" << n << ")"; }
void GVarTerm::print(std::ostream &out) const    { out << *ref->name; }

// }}}
// {{{ definition of G*Term::sig

FWSignature GValTerm::sig() const      { return val.sig(); }
FWSignature GFunctionTerm::sig() const { return FWSignature(name, args.size()); }
FWSignature GLinearTerm::sig() const   { throw std::logic_error("must not be called"); }
FWSignature GVarTerm::sig() const      { throw std::logic_error("must not be called"); }

// }}}
// {{{ definition of G*Term::eval

GTerm::EvalResult GValTerm::eval() const      { return EvalResult(true, val); }
GTerm::EvalResult GFunctionTerm::eval() const { return EvalResult(false, Value()); }
GTerm::EvalResult GLinearTerm::eval() const   { return EvalResult(false, Value()); }
GTerm::EvalResult GVarTerm::eval() const      { return EvalResult(false, Value()); }

// }}}
// {{{ definition of G*Term::occurs

bool GValTerm::occurs(GRef &) const { return false; }
bool GFunctionTerm::occurs(GRef &x) const { 
    for (auto &y : args) {
        if (y->occurs(x)) { return true; }
    }
    return false;
}
bool GLinearTerm::occurs(GRef &x) const { return ref->occurs(x); }
bool GVarTerm::occurs(GRef &x) const { return ref->occurs(x); }

// }}}
// {{{ definition of G*Term::reset

void GValTerm::reset() { }
void GFunctionTerm::reset() { 
    for (auto &y : args) { y->reset(); }
}
void GLinearTerm::reset() { ref->reset(); }
void GVarTerm::reset()    { ref->reset(); }

// }}}
// {{{ definition of G*Term::match

bool GValTerm::match(Value const &x) { return val == x; }
bool GFunctionTerm::match(Value const &x) {
    if (x.type() != Value::FUNC || sig() != x.sig()) { return false; }
    else {
        auto it = args.begin();
        for (auto &y : x.args()) {
            if (!(*it++)->match(y)) { return false; }
        }
        return true;
    }
}

bool GLinearTerm::match(Value const &x) {
    if (x.type() != Value::NUM) { return false; }
    else {
        int y = x.num();
        y-= n;
        if (y % m != 0) { return false; }
        else {
            y /= m;
            if (*ref) { return ref->match(Value(y)); }
            else {
                *ref = Value(y);
                return true;
            }
        }
    }
}
bool GVarTerm::match(Value const &x) {
    if (*ref) { return ref->match(x); }
    else {
        *ref = x;
        return true;
    }
}

// }}}
// {{{ definition of G*Term::unify(GTerm &)

bool GValTerm::unify(GTerm &x)      { return x.match(val); }
bool GFunctionTerm::unify(GTerm &x) { return x.unify(*this); }
bool GLinearTerm::unify(GTerm &x)   { return x.unify(*this); }
bool GVarTerm::unify(GTerm &x)      { return x.unify(*this); }

// }}}
// {{{ definition of G*Term::unify(GFunctionTerm &)


bool GValTerm::unify(GFunctionTerm &x) { return x.match(val); }
bool GFunctionTerm::unify(GFunctionTerm &x) { 
    if (name != x.name || args.size() != x.args.size()) { return false; }
    else {
        for (auto it = args.begin(), jt = x.args.begin(), ie = args.end(); it != ie; ++it, ++jt) {
            if (!(*it)->unify(**jt)) { return false; }
        }
        return true;
    }
}
bool GLinearTerm::unify(GFunctionTerm &) { return false; }
bool GVarTerm::unify(GFunctionTerm &x) { 
    if (*ref) { return ref->unify(x); }
    else if (!x.occurs(*ref)) {
        *ref = x;
        return true;
    }
    else { return false; }
}

// }}}
// {{{ definition of G*Term::unify(GLinearTerm &)

bool GValTerm::unify(GLinearTerm &x)     { return x.match(val); }
bool GFunctionTerm::unify(GLinearTerm &) { return false; }
bool GLinearTerm::unify(GLinearTerm &) { 
    // Note: more could be done but this would be somewhat involved
    //       because this would require rational numbers
    //       as of now this simply unifies too much
    return true;
}
bool GVarTerm::unify(GLinearTerm &x) { 
    if (*ref) { return ref->unify(x); }
    else {
        // see not at: GLinearTerm::unify(GLinearTerm &x)
        return true;
    }
}

// }}}
// {{{ definition of G*Term::unify(GVarTerm &)

bool GValTerm::unify(GVarTerm &x) { return x.match(val); }
bool GFunctionTerm::unify(GVarTerm &x) {
    if (*x.ref) { return x.ref->unify(*this); }
    else if (!occurs(*x.ref)) {
        *x.ref = *this;
        return true;
    }
    else { return false; }
}
bool GLinearTerm::unify(GVarTerm &x) { 
    if (*x.ref) { return x.ref->unify(*this); }
    else {
        // see not at: GLinearTerm::unify(GLinearTerm &x)
        return true;
    }
}
bool GVarTerm::unify(GVarTerm &x) { 
    if (*ref)        { return ref->unify(x); }
    else if (*x.ref) { return x.ref->unify(*this); }
    else if (ref->name != x.ref->name) {
        *ref = x;
        return true;
    }
    else { return true; }
}

// }}}
// {{{ definition of G*Term::~G*Term

GValTerm::~GValTerm() { }
GFunctionTerm::~GFunctionTerm() { }
GLinearTerm::~GLinearTerm() { }
GVarTerm::~GVarTerm() { }

// }}}

// {{{ definition of Term::rename

void PoolTerm::rename(FWString) {
    throw std::logic_error("must not be called");
}
void ValTerm::rename(FWString x) { 
    value = Value(x);
}
void LinearTerm::rename(FWString) {
    throw std::logic_error("must not be called");
}
void VarTerm::rename(FWString) {
    throw std::logic_error("must not be called");
}
void UnOpTerm::rename(FWString) {
    throw std::logic_error("must not be called");
}
void BinOpTerm::rename(FWString) {
    throw std::logic_error("must not be called");
}
void DotsTerm::rename(FWString) {
    throw std::logic_error("must not be called");
}
void LuaTerm::rename(FWString) {
    throw std::logic_error("must not be called");
}
void FunctionTerm::rename(FWString x) {
    name = x;
}

// }}}
// {{{ definition of Term::getLevel

unsigned PoolTerm::getLevel() const {
    unsigned level = 0;
    for (auto &x : args) { level = std::max(x->getLevel(), level); }
    return level;
}
unsigned ValTerm::getLevel() const {
    return 0;
}
unsigned LinearTerm::getLevel() const {
    return var->getLevel();
}
unsigned VarTerm::getLevel() const {
    return level;
}
unsigned UnOpTerm::getLevel() const {
    return arg->getLevel();
}
unsigned BinOpTerm::getLevel() const {
    return std::max(left->getLevel(), right->getLevel());
}
unsigned DotsTerm::getLevel() const {
    return std::max(left->getLevel(), right->getLevel());
}
unsigned LuaTerm::getLevel() const {
    unsigned level = 0;
    for (auto &x : args) { level = std::max(x->getLevel(), level); }
    return level;
}
unsigned FunctionTerm::getLevel() const {
    unsigned level = 0;
    for (auto &x : args) { level = std::max(x->getLevel(), level); }
    return level;
}

// }}}
// {{{ definition of Term::isNotNumeric

bool PoolTerm::isNotNumeric() const     { return false; }
bool ValTerm::isNotNumeric() const      { return value.type() != Value::NUM; }
bool VarTerm::isNotNumeric() const      { return false; }
bool LinearTerm::isNotNumeric() const   { return false; }
bool UnOpTerm::isNotNumeric() const     { return false; }
bool BinOpTerm::isNotNumeric() const    { return false; }
bool DotsTerm::isNotNumeric() const     { return false; }
bool LuaTerm::isNotNumeric() const      { return false; }
bool FunctionTerm::isNotNumeric() const { return true; }

// }}}
// {{{ definition of Term::getInvertibility

Term::Invertibility PoolTerm::getInvertibility() const     { return Term::NOT_INVERTIBLE; }
Term::Invertibility ValTerm::getInvertibility() const      { return Term::CONSTANT; }
Term::Invertibility VarTerm::getInvertibility() const      { return Term::INVERTIBLE; }
Term::Invertibility LinearTerm::getInvertibility() const   { return Term::INVERTIBLE; }
Term::Invertibility UnOpTerm::getInvertibility() const     { return Term::NOT_INVERTIBLE; }
Term::Invertibility BinOpTerm::getInvertibility() const    { return Term::NOT_INVERTIBLE; }
Term::Invertibility DotsTerm::getInvertibility() const     { return Term::NOT_INVERTIBLE; }
Term::Invertibility LuaTerm::getInvertibility() const      { return Term::NOT_INVERTIBLE; }
Term::Invertibility FunctionTerm::getInvertibility() const { return Term::INVERTIBLE; }

// }}}
// {{{ definition of Term::print

void PoolTerm::print(std::ostream &out) const    { print_comma(out, args, ";", [](std::ostream &out, UTerm const &y) { out << *y; }); }
void ValTerm::print(std::ostream &out) const     { out << value; }
void VarTerm::print(std::ostream &out) const     { out << *(name); }
void LinearTerm::print(std::ostream &out) const  { out << "(" << m << "*" << *var << "+" << n << ")"; }
void UnOpTerm::print(std::ostream &out) const {
    if (op == UnOp::ABS) { 
        out << "|" << *arg << "|";
    }
    else { 
        out << "(" << op << *arg << ")";
    }
}
void BinOpTerm::print(std::ostream &out) const {
    out << "(" << *left << op << *right << ")";
}
void DotsTerm::print(std::ostream &out) const {
    out << "(" << *left << ".." << *right << ")";
}
void LuaTerm::print(std::ostream &out) const {
    out << "@" << *name << "(";
    print_comma(out, args, ",", [](std::ostream &out, UTerm const &y) { out << *y; });
    out << ")";
}
void FunctionTerm::print(std::ostream &out) const {
    out << *(name) << "(";
    print_comma(out, args, ",", [](std::ostream &out, UTerm const &y) { out << *y; });
    out << ")";
}

// }}}
// {{{ definition of Term::simplify

Term::SimplifyRet::SimplifyRet(SimplifyRet &&x) : type(x.type) {
    switch(type) {
        case LINEAR:
        case REPLACE:   { x.type = UNTOUCHED; }
        case UNTOUCHED: { 
            term = x.term;
            break;
        }
        case CONSTANT: {
            val = x.val;
            break;
        }
    }
}
//! Reference to untouched term.
Term::SimplifyRet::SimplifyRet(Term &x, bool project) : type(UNTOUCHED), project(project), term(&x) { }
//! Indicate replacement with linear term.
Term::SimplifyRet::SimplifyRet(std::unique_ptr<LinearTerm> &&x) : type(LINEAR), project(false), term(x.release()) { }
//! Indicate replacement with arbitrary term.
Term::SimplifyRet::SimplifyRet(UTerm &&x) : type(REPLACE), project(false), term(x.release()) { }
//! Indicate replacement with value.
Term::SimplifyRet::SimplifyRet(Value const &x) : type(CONSTANT), project(false), val(x) { }
bool Term::SimplifyRet::notNumeric() const {
    switch (type) {
        case LINEAR:    { return false; }
        case CONSTANT:  { return val.type() != Value::NUM; }
        case REPLACE:
        case UNTOUCHED: { return term->isNotNumeric(); }
    }
    assert(false);
    return false;
}
bool Term::SimplifyRet::constant() const  { return type == CONSTANT; }
bool Term::SimplifyRet::isZero() const    { return constant() && val.type() == Value::NUM && val.num() == 0; }
LinearTerm &Term::SimplifyRet::lin()      { return static_cast<LinearTerm&>(*term); }
Term::SimplifyRet &Term::SimplifyRet::update(UTerm &x) {
    switch (type) {
        case CONSTANT: { 
            x = make_locatable<ValTerm>(x->loc(), val);
            return *this;
        }
        case LINEAR: { 
            if (lin().m == 1 && lin().n == 0) { 
                type = UNTOUCHED;
                x = std::move(lin().var); 
                delete term;
                return *this;
            }
        }
        case REPLACE:  { 
            type = UNTOUCHED;
            x.reset(term);
            return *this;
        }
        default: { return *this; }
    }
}
Term::SimplifyRet::~SimplifyRet() {
    if (type == LINEAR || type == REPLACE) { delete term; }
}

Term::SimplifyRet PoolTerm::simplify(DotsMap &, ScriptMap &, unsigned &, bool, bool)   { return {*this, false}; }
Term::SimplifyRet ValTerm::simplify(DotsMap &, ScriptMap &, unsigned &, bool, bool)    { return {value}; }
Term::SimplifyRet LinearTerm::simplify(DotsMap &, ScriptMap &, unsigned &, bool, bool) { return {*this, false}; }
Term::SimplifyRet VarTerm::simplify(DotsMap &, ScriptMap &, unsigned &auxNum, bool positional, bool arithmetic) {
    if (name == "_") {
        ref = std::make_shared<Value>();
        if (positional) { return {*this, true}; }
        else { name = Term::uniqueName(auxNum, "#Anon"); }
    }
    if (arithmetic) { return {make_locatable<LinearTerm>(loc(), *this, 1, 0)}; }
    else            { return {*this, false}; }
}
Term::SimplifyRet UnOpTerm::simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool, bool) {
    SimplifyRet ret(arg->simplify(dots, scripts, auxNum, false, true));
    if(ret.notNumeric()) {
        GRINGO_REPORT(W_TERM_UNDEFINED) 
            << loc() << ": warning: operation undefined, a zero is substituted:\n"
            << "  " << *this << "\n";
        return {Value(0)};
    }
    switch (ret.type) {
        case SimplifyRet::CONSTANT: { return {Gringo::eval(op, ret.val.num())}; }
        case SimplifyRet::LINEAR: {
            if (op == UnOp::NEG) { 
                ret.lin().m *= -1;
                ret.lin().n *= -1;
                return ret;
            }
        }
        default: {
            ret.update(arg);
            return {*this, false};
        }
    }
}
Term::SimplifyRet BinOpTerm::simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool, bool) {
    auto retLeft(left->simplify(dots, scripts, auxNum, false, true));
    auto retRight(right->simplify(dots, scripts, auxNum, false, true));
    if (retLeft.notNumeric() || retRight.notNumeric() || (op == BinOp::DIV && retRight.isZero())) {
        GRINGO_REPORT(W_TERM_UNDEFINED) 
            << loc() << ": warning: operation undefined, a zero is substituted:\n"
            << "  " << *this << "\n";
        return {Value(0)};
    }
    else if (op == BinOp::MUL && (retLeft.isZero() || retRight.isZero())) {
        return {Value(0)};
    } 
    else if (retLeft.type == SimplifyRet::CONSTANT && retRight.type == SimplifyRet::CONSTANT) {
        return {Gringo::eval(op, retLeft.val.num(), retRight.val.num())};
    }
    else if (retLeft.type == SimplifyRet::CONSTANT && retRight.type == SimplifyRet::LINEAR) {
        if (op == BinOp::ADD) {
            retRight.lin().n += retLeft.val.num();
            return retRight;
        }
        else if (op == BinOp::SUB) {
            retRight.lin().n = retLeft.val.num() - retRight.lin().n;
            retRight.lin().m = -retRight.lin().m;
            return retRight;
        }
        else if (op == BinOp::MUL) {
            retRight.lin().n *= retLeft.val.num();
            retRight.lin().m *= retLeft.val.num();
            return retRight;
        }
    }
    else if (retLeft.type == SimplifyRet::LINEAR && retRight.type == SimplifyRet::CONSTANT) {
        if (op == BinOp::ADD) {
            retLeft.lin().n += retRight.val.num();
            return retLeft;
        }
        else if (op == BinOp::SUB) {
            retLeft.lin().n-= retRight.val.num();
            return retLeft;
        }
        else if (op == BinOp::MUL) {
            retLeft.lin().n *= retRight.val.num();
            retLeft.lin().m *= retRight.val.num();
            return retLeft;
        }
    }
    retLeft.update(left);
    retRight.update(right);
    return {*this, false};
}
Term::SimplifyRet DotsTerm::simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool, bool) {
    left->simplify(dots, scripts, auxNum, false, false).update(left);
    right->simplify(dots, scripts, auxNum, false, false).update(right);
    return { LinearTerm::create(loc(), dots, auxNum, std::move(left), std::move(right)) };
}
Term::SimplifyRet LuaTerm::simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool, bool) {
    for (auto &arg : args) { arg->simplify(dots, scripts, auxNum, false, false).update(arg); }
    return { LinearTerm::create(loc(), scripts, auxNum, std::move(name), std::move(args)) };
}
Term::SimplifyRet FunctionTerm::simplify(DotsMap &dots, ScriptMap &scripts, unsigned &auxNum, bool positional, bool) {
    bool constant  = true;
    bool projected = false;
    for (auto &arg : args) { 
        auto ret(arg->simplify(dots, scripts, auxNum, positional, false));
        constant  = constant  && ret.constant();
        projected = projected || ret.project;
        ret.update(arg);
    }
    if (constant) { return {eval()}; }
    else          { return {*this, projected}; }
}

// }}}
// {{{ definition of Term::project

// Notes on return value:
//   first:  term to replace with
//           projected variables are stripped
//           null if untouched
//   second: term with variables renamed
//           projected if null
//   third:  term with variables renamed
//           projected variables are renamed
//           never null

UTerm wrap(UTerm &&x) {
    UTermVec args;
    args.emplace_back(std::move(x));
    return make_locatable<FunctionTerm>(args.front()->loc(), "#b", std::move(args));
}

Term::ProjectRet PoolTerm::project(bool, unsigned &) {
    throw std::logic_error("Term::project must be called after Term::unpool");
}
Term::ProjectRet ValTerm::project(bool rename, unsigned &) { 
    assert(!rename); (void)rename;
    return std::make_tuple(nullptr, UTerm(clone()), UTerm(clone()));
}
Term::ProjectRet LinearTerm::project(bool rename, unsigned &auxNum) {
    assert(!rename); (void)rename;
    UTerm y(uniqueVar(loc(), auxNum, 0, "#X"));
    UTerm x(wrap(UTerm(y->clone())));
    return std::make_tuple(wrap(make_locatable<LinearTerm>(loc(), std::move(var), m, n)), std::move(x), std::move(y));
}
Term::ProjectRet VarTerm::project(bool rename, unsigned &auxNum) {
    assert(!rename); (void)rename;
    if (*name == "_") { 
        UTerm r(make_locatable<ValTerm>(loc(), "#p"));
        UTerm x(r->clone());
        UTerm y(uniqueVar(loc(), auxNum, 0, "#P"));
        return std::make_tuple(std::move(r), std::move(x), std::move(y));
    }
    else {
        UTerm y(uniqueVar(loc(), auxNum, 0, "#X"));
        UTerm x(wrap(UTerm(y->clone())));
        return std::make_tuple(wrap(UTerm(clone())), std::move(x), std::move(y));
    }
}
Term::ProjectRet UnOpTerm::project(bool rename, unsigned &auxNum) {
    assert(!rename); (void)rename;
    UTerm y(uniqueVar(loc(), auxNum, 0, "#X"));
    UTerm x(wrap(UTerm(y->clone())));
    return std::make_tuple(wrap(make_locatable<UnOpTerm>(loc(), op, std::move(arg))), std::move(x), std::move(y));
}
Term::ProjectRet BinOpTerm::project(bool rename, unsigned &auxNum) {
    assert(!rename); (void)rename;
    UTerm y(uniqueVar(loc(), auxNum, 0, "#X"));
    UTerm x(wrap(UTerm(y->clone())));
    return std::make_tuple(wrap(make_locatable<BinOpTerm>(loc(), op, std::move(left), std::move(right))), std::move(x), std::move(y));
}
Term::ProjectRet DotsTerm::project(bool, unsigned &) {
    throw std::logic_error("Term::project must be called after Term::simplify");
}
Term::ProjectRet LuaTerm::project(bool rename, unsigned &auxNum) {
    assert(!rename); (void)rename;
    UTerm y(uniqueVar(loc(), auxNum, 0, "#X"));
    UTerm x(wrap(UTerm(y->clone())));
    return std::make_tuple(make_locatable<LuaTerm>(loc(), name, std::move(args)), std::move(x), std::move(y));
}
Term::ProjectRet FunctionTerm::project(bool rename, unsigned &auxNum) {
    UTermVec argsProjected;
    UTermVec argsProject;
    for (auto &arg : args) {
        auto ret(arg->project(false, auxNum));
        Term::replace(arg, std::move(std::get<0>(ret)));
        argsProjected.emplace_back(std::move(std::get<1>(ret)));
        argsProject.emplace_back(std::move(std::get<2>(ret)));
    }
    FWString oldName = name;
    if (rename) { name = FWString("#p_" + *name); }
    return std::make_tuple(
            nullptr,  
            make_locatable<FunctionTerm>(loc(), name, std::move(argsProjected)),
            make_locatable<FunctionTerm>(loc(), oldName, std::move(argsProject)));
}

// }}}
// {{{ definition of Term::hasVar

bool PoolTerm::hasVar() const {
    for (auto &x : args) {
        if (x->hasVar()) { return true; }
    }
    return false;
}
bool ValTerm::hasVar() const { 
    return false;
}
bool LinearTerm::hasVar() const {
    return true;
}
bool VarTerm::hasVar() const {
    return true;
}
bool UnOpTerm::hasVar() const {
    return arg->hasVar();
}
bool BinOpTerm::hasVar() const {
    return left->hasVar() || right->hasVar();
}
bool DotsTerm::hasVar() const {
    return left->hasVar() || right->hasVar();
}
bool LuaTerm::hasVar() const {
    for (auto &x : args) {
        if (x->hasVar()) { return true; }
    }
    return false;
}
bool FunctionTerm::hasVar() const {
    for (auto &x : args) {
        if (x->hasVar()) { return true; }
    }
    return false;
}

// }}}
// {{{ definition of Term::hasPool

bool PoolTerm::hasPool() const   { return true; }
bool ValTerm::hasPool() const    { return false; }
bool LinearTerm::hasPool() const { return false; }
bool VarTerm::hasPool() const    { return false; }
bool UnOpTerm::hasPool() const   { return arg->hasPool(); }
bool BinOpTerm::hasPool() const  { return left->hasPool() || right->hasPool(); }
bool DotsTerm::hasPool() const   { return left->hasPool() || right->hasPool(); }
bool LuaTerm::hasPool() const {
    for (auto &x : args) { if (x->hasPool()) { return true; } }
    return false;
}
bool FunctionTerm::hasPool() const {
    for (auto &x : args) { if (x->hasPool()) { return true; } }
    return false;
}

// }}}
// {{{ definition of Term::collect

void PoolTerm::collect(VarTermBoundVec &vars, bool bound) const {
    for (auto &y : args) { y->collect(vars, bound); }
}
void ValTerm::collect(VarTermBoundVec &, bool) const {
}
void LinearTerm::collect(VarTermBoundVec &vars, bool bound) const {
    var->collect(vars, bound);
}
void VarTerm::collect(VarTermBoundVec &vars, bool bound) const {
    vars.emplace_back(const_cast<VarTerm*>(this), bound);
}
void UnOpTerm::collect(VarTermBoundVec &vars, bool) const {
    arg->collect(vars, false);
}
void BinOpTerm::collect(VarTermBoundVec &vars, bool) const {
    left->collect(vars, false);
    right->collect(vars, false);
}
void DotsTerm::collect(VarTermBoundVec &vars, bool) const {
    left->collect(vars, false);
    right->collect(vars, false);
}
void LuaTerm::collect(VarTermBoundVec &vars, bool) const {
    for (auto &y : args) { y->collect(vars, false); }
}
void FunctionTerm::collect(VarTermBoundVec &vars, bool bound) const {
    for (auto &y : args) { y->collect(vars, bound); }
}

// }}}
// {{{ definition of Term::collect

void PoolTerm::collect(VarSet &vars, unsigned minLevel , unsigned maxLevel) const {
    for (auto &y : args) { y->collect(vars, minLevel, maxLevel); }
}
void ValTerm::collect(VarSet &, unsigned, unsigned) const {
}
void LinearTerm::collect(VarSet &vars, unsigned minLevel , unsigned maxLevel) const {
    var->collect(vars, minLevel, maxLevel);
}
void VarTerm::collect(VarSet &vars, unsigned minLevel , unsigned maxLevel) const {
    if (minLevel <= level && level <= maxLevel) { vars.emplace(name); }
}
void UnOpTerm::collect(VarSet &vars, unsigned minLevel , unsigned maxLevel) const {
    arg->collect(vars, minLevel, maxLevel);
}
void BinOpTerm::collect(VarSet &vars, unsigned minLevel , unsigned maxLevel) const {
    left->collect(vars, minLevel, maxLevel);
    right->collect(vars, minLevel, maxLevel);
}
void DotsTerm::collect(VarSet &vars, unsigned minLevel , unsigned maxLevel) const {
    left->collect(vars, minLevel, maxLevel);
    right->collect(vars, minLevel, maxLevel);
}
void LuaTerm::collect(VarSet &vars, unsigned minLevel , unsigned maxLevel) const {
    for (auto &y : args) { y->collect(vars, minLevel, maxLevel); }
}
void FunctionTerm::collect(VarSet &vars, unsigned minLevel , unsigned maxLevel) const {
    for (auto &y : args) { y->collect(vars, minLevel, maxLevel); }
}

// }}}
// {{{ definition of Term::eval

Value PoolTerm::eval() const { throw std::logic_error("Term::unpool must be called before Term::eval"); }
Value ValTerm::eval() const { return value; }
Value VarTerm::eval() const { return *ref; }
Value LinearTerm::eval() const {
	Value value = var->eval();
	if (value.type() == Value::NUM) { return {m * value.num() + n}; }
	else {
		GRINGO_REPORT(W_TERM_UNDEFINED)
			<< loc() << ": warning: operation undefined, a zero is substituted:\n"
			<< "  " << *this << "\n";
		return {0};
	}
}
Value UnOpTerm::eval() const {
	Value value = arg->eval();
	if (value.type() == Value::NUM) {
		int num = value.num();
		switch (op) {
			case UnOp::NEG: { return {Value(-num)}; }
			case UnOp::ABS: { return {std::abs(num)}; }
			case UnOp::NOT: { return {~num}; }
		}
		assert(false);
		return {0};
	}
	else {
		GRINGO_REPORT(W_TERM_UNDEFINED)
			<< loc() << ": warning: operation undefined, a zero is substituted:\n"
			<< "  " << *this << "\n";
		return {0};
	}
}
Value BinOpTerm::eval() const {
	Value l(left->eval());
	Value r(right->eval());
	if (l.type() == Value::NUM && r.type() == Value::NUM && (op != BinOp::DIV || r.num() != 0)) { return {Gringo::eval(op, l.num(), r.num())}; }
	else {
		GRINGO_REPORT(W_TERM_UNDEFINED)
			<< loc() << ": warning: operation undefined, a zero is substituted:\n"
			<< "  " << *this << "\n";
		return {0};
	}
}
Value DotsTerm::eval() const { throw std::logic_error("Term::rewriteDots must be called before Term::eval"); }
Value LuaTerm::eval() const { throw std::logic_error("TODO: lua support is future music"); }
Value FunctionTerm::eval() const {
	cache.clear();
	for (auto &term : args) { cache.emplace_back(term->eval()); }
	return Value(name, cache);
}

// }}}
// {{{ definition of Term::bind

bool Term::bind(VarSet &bound) {
    VarTermBoundVec occs;
    collect(occs, false);
    bool ret = false;
    for (auto &x : occs) { 
        if ((x.first->bindRef = bound.insert(x.first->name).second)) { ret = true; }
    }
    return ret;
}

// }}}
// {{{ definition of Term::match

bool PoolTerm::match(Value const &) const { throw std::logic_error("Term::unpool must be called before Term::match"); }
bool ValTerm::match(Value const &x) const { return value == x; }
bool VarTerm::match(Value const &x) const {
    if (bindRef) {
        *ref = x;
        return true;
    }
    else { return x == *ref; }
}
bool LinearTerm::match(Value const &x) const {
	if (x.type() == Value::NUM) {
		assert(m != 0);
		int c(x.num() - n);
		if (c % m == 0) { return var->match({c/m}); }
	}
	return false;
}
bool UnOpTerm::match(Value const &) const  { throw std::logic_error("Term::rewriteArithmetics must be called before Term::match"); }
bool BinOpTerm::match(Value const &) const { throw std::logic_error("Term::rewriteArithmetics must be called before Term::match"); }
bool DotsTerm::match(Value const &) const  { throw std::logic_error("Term::rewriteDots must be called before Term::match"); }
bool LuaTerm::match(Value const &) const   { throw std::logic_error("Term::rewriteArithmetics must be called before Term::match"); }
bool FunctionTerm::match(Value const &x) const {
	if (x.type() == Value::FUNC) {
		Signature s(*x.sig());
		if (s.name() == name && s.length() == args.size()) {
			auto it = x.args().begin();
			for (auto &term : args) {
				if (!term->match(*it++)) { return false; }
			}
			return true;
		}
	}
	return false;
}

// }}}
// {{{ definition of Term::unpool

void PoolTerm::unpool(UTermVec &x) const { 
    for (auto &t : args) { t->unpool(x); }
}
void ValTerm::unpool(UTermVec &x) const {
    x.emplace_back(UTerm(clone()));
}
void LinearTerm::unpool(UTermVec &x) const {
    x.emplace_back(UTerm(clone()));
}
void VarTerm::unpool(UTermVec &x) const {
    x.emplace_back(UTerm(clone()));
}
void UnOpTerm::unpool(UTermVec &x) const {
    auto f = [&](UTerm &&y) { x.emplace_back(make_locatable<UnOpTerm>(loc(), op, std::move(y))); };
    Term::unpool(arg, Gringo::unpool, f);
}
void BinOpTerm::unpool(UTermVec &x) const {
    auto f = [&](UTerm &&l, UTerm &&r) { x.emplace_back(make_locatable<BinOpTerm>(loc(), op, std::move(l), std::move(r))); };
    Term::unpool(left, right, Gringo::unpool, Gringo::unpool, f);
}
void DotsTerm::unpool(UTermVec &x) const {
    auto f = [&](UTerm &&l, UTerm &&r) { x.emplace_back(make_locatable<DotsTerm>(loc(), std::move(l), std::move(r))); };
    Term::unpool(left, right, Gringo::unpool, Gringo::unpool, f);
}
void LuaTerm::unpool(UTermVec &x) const {
    auto f = [&](UTermVec &&args) { x.emplace_back(make_locatable<LuaTerm>(loc(), name, std::move(args))); };
    Term::unpool(args.begin(), args.end(), Gringo::unpool, f);
}
void FunctionTerm::unpool(UTermVec &x) const {
    auto f = [&](UTermVec &&args) { x.emplace_back(make_locatable<FunctionTerm>(loc(), name, std::move(args))); };
    Term::unpool(args.begin(), args.end(), Gringo::unpool, f);
}

// }}}
// {{{ definition of Term::rewriteArithmetics

UTerm PoolTerm::rewriteArithmetics(Term::ArithmeticsMap &, unsigned &) {
    throw std::logic_error("Term::rewriteArithmetics must be called before Term::rewriteArithmetics");
}
UTerm ValTerm::rewriteArithmetics(Term::ArithmeticsMap &, unsigned &) { return nullptr; }
UTerm VarTerm::rewriteArithmetics(Term::ArithmeticsMap &, unsigned &) { return nullptr; }
UTerm UnOpTerm::rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum) {
    return Term::insert(arith, auxNum, make_locatable<UnOpTerm>(loc(), op, std::move(arg)));
}
UTerm BinOpTerm::rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum) {
    return Term::insert(arith, auxNum, make_locatable<BinOpTerm>(loc(), op, std::move(left), std::move(right)));
}
UTerm DotsTerm::rewriteArithmetics(Term::ArithmeticsMap &, unsigned &) {
    throw std::logic_error("Term::rewriteDots must be called before Term::rewriteArithmetics");
}
UTerm LuaTerm::rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum) {
    return Term::insert(arith, auxNum, make_locatable<LuaTerm>(loc(), name, std::move(args)));
}
UTerm FunctionTerm::rewriteArithmetics(Term::ArithmeticsMap &arith, unsigned &auxNum) {
    for (auto &arg : args) { Term::replace(arg, arg->rewriteArithmetics(arith, auxNum)); }
    return nullptr;
}
UTerm LinearTerm::rewriteArithmetics(Term::ArithmeticsMap &, unsigned &) { return nullptr; }

// }}}
// {{{ definition of Term::operator==

bool PoolTerm::operator==(Term const &x) const { 
    auto t = dynamic_cast<PoolTerm const*>(&x);
    return t && is_value_equal_to(args, t->args);
}
bool ValTerm::operator==(Term const &x) const {
    auto t = dynamic_cast<ValTerm const*>(&x);
    return t && value == t->value;
}
bool VarTerm::operator==(Term const &x) const {
    auto t = dynamic_cast<VarTerm const*>(&x);
    return t && *name == *t->name && level == t->level;
}
bool LinearTerm::operator==(Term const &x) const {
    auto t = dynamic_cast<LinearTerm const*>(&x);
    return t && m == t->m && n == t->n && is_value_equal_to(var, t->var);
}
bool UnOpTerm::operator==(Term const &x) const {
    auto t = dynamic_cast<UnOpTerm const*>(&x);
    return t && op == t->op && is_value_equal_to(arg, t->arg);
}
bool BinOpTerm::operator==(Term const &x) const {
    auto t = dynamic_cast<BinOpTerm const*>(&x);
    return t && op == t->op && is_value_equal_to(left, t->left) && is_value_equal_to(right, t->right);
}
bool DotsTerm::operator==(Term const &) const {
    // Note: each DotsTerm is associated to a unique variable
    return false;
}
bool LuaTerm::operator==(Term const &x) const {
    auto t = dynamic_cast<LuaTerm const*>(&x);
    return t && *name == *t->name && is_value_equal_to(args, t->args);
}
bool FunctionTerm::operator==(Term const &x) const {
    auto t = dynamic_cast<FunctionTerm const*>(&x);
    return t && *name == *t->name && is_value_equal_to(args, t->args);
}

// }}}
// {{{ definition of Term::hash

size_t PoolTerm::hash() const { 
    return get_value_hash(typeid(PoolTerm).hash_code(), args);
}
size_t ValTerm::hash() const {
    return get_value_hash(typeid(ValTerm).hash_code(), value);
}
size_t VarTerm::hash() const {
    return get_value_hash(typeid(VarTerm).hash_code(), *name, level);
}
size_t LinearTerm::hash() const {
    return get_value_hash(typeid(LinearTerm).hash_code(), m, n, var->hash());
}
size_t UnOpTerm::hash() const {
    return get_value_hash(typeid(UnOpTerm).hash_code(), size_t(op), arg);
}
size_t BinOpTerm::hash() const {
    return get_value_hash(typeid(BinOpTerm).hash_code(), size_t(op), left, right);
}
size_t DotsTerm::hash() const {
    return get_value_hash(typeid(DotsTerm).hash_code(), left, right);
}
size_t LuaTerm::hash() const {
    return get_value_hash(typeid(LuaTerm).hash_code(), *name, args);
}
size_t FunctionTerm::hash() const {
    return get_value_hash(typeid(FunctionTerm).hash_code(), *name, args);
}

// }}}
// {{{ definition of Term::clone

PoolTerm *PoolTerm::clone() const { 
    return make_locatable<PoolTerm>(loc(), get_clone(args)).release();
}
ValTerm *ValTerm::clone() const {
    return make_locatable<ValTerm>(loc(), value).release();
}
VarTerm *VarTerm::clone() const {
    return make_locatable<VarTerm>(loc(), name, ref, level, bindRef).release();
}
LinearTerm *LinearTerm::clone() const {
    return make_locatable<LinearTerm>(loc(), *var, m, n).release();
}
UnOpTerm *UnOpTerm::clone() const {
    return make_locatable<UnOpTerm>(loc(), op, get_clone(arg)).release();
}
BinOpTerm *BinOpTerm::clone() const {
    return make_locatable<BinOpTerm>(loc(), op, get_clone(left), get_clone(right)).release();
}
DotsTerm *DotsTerm::clone() const {
    return make_locatable<DotsTerm>(loc(), get_clone(left), get_clone(right)).release();
}
LuaTerm *LuaTerm::clone() const {
    return make_locatable<LuaTerm>(loc(), name, get_clone(args)).release();
}
FunctionTerm *FunctionTerm::clone() const {
    return make_locatable<FunctionTerm>(loc(), name, get_clone(args)).release();
}

// }}}
// {{{ definition of Term::getSig

FWSignature PoolTerm::getSig() const { assert(false); throw std::logic_error("Term::getSig must not be called on PoolTerm"); }
FWSignature ValTerm::getSig() const { 
    switch (value.type()) {
        case Value::ID:
        case Value::FUNC: { return value.sig(); }
        default:          { throw std::logic_error("Term::getSig must not be called on ValTerm"); }
    }
}
FWSignature LinearTerm::getSig() const   { throw std::logic_error("Term::getSig must not be called on LinearTerm"); }
FWSignature VarTerm::getSig() const      { throw std::logic_error("Term::getSig must not be called on VarTerm"); }
FWSignature UnOpTerm::getSig() const     { throw std::logic_error("Term::getSig must not be called on UnOpTerm"); }
FWSignature BinOpTerm::getSig() const    { throw std::logic_error("Term::getSig must not be called on DotsTerm"); }
FWSignature DotsTerm::getSig() const     { throw std::logic_error("Term::getSig must not be called on LuaTerm"); }
FWSignature LuaTerm::getSig() const      { return FWSignature(name, args.size()); }
FWSignature FunctionTerm::getSig() const { return FWSignature(name, args.size()); }

// }}}
// {{{ definition of Term::renameVars

UTerm PoolTerm::renameVars(RenameMap &names) const { 
    UTermVec args;
    for (auto &x : this->args) { args.emplace_back(x->renameVars(names)); }
    return make_locatable<PoolTerm>(loc(), std::move(args));
}
UTerm ValTerm::renameVars(RenameMap &) const { return UTerm(clone()); }
UTerm LinearTerm::renameVars(RenameMap &names) const { 
    return make_locatable<LinearTerm>(loc(), UVarTerm(static_cast<VarTerm*>(var->renameVars(names).release())), m, n);
}
UTerm VarTerm::renameVars(RenameMap &names) const { 
    auto ret(names.emplace(name, std::make_pair(name, nullptr)));
    if (ret.second) { 
        ret.first->second.first  = (bindRef ? "X" : "Y") + std::to_string(names.size() - 1); 
        ret.first->second.second = std::make_shared<Value>();
    }
    return make_locatable<VarTerm>(loc(), ret.first->second.first, ret.first->second.second, 0, bindRef);
}
UTerm UnOpTerm::renameVars(RenameMap &names) const { return make_locatable<UnOpTerm>(loc(), op, arg->renameVars(names)); }
UTerm BinOpTerm::renameVars(RenameMap &names) const {
    UTerm term(left->renameVars(names));
    return make_locatable<BinOpTerm>(loc(), op, std::move(term), right->renameVars(names));
}
UTerm DotsTerm::renameVars(RenameMap &names) const {
    UTerm term(left->renameVars(names));
    return make_locatable<DotsTerm>(loc(), std::move(term), right->renameVars(names));
}
UTerm LuaTerm::renameVars(RenameMap &names) const {
    UTermVec args;
    for (auto &x : this->args) { args.emplace_back(x->renameVars(names)); }
    return make_locatable<LuaTerm>(loc(), name, std::move(args));
}
UTerm FunctionTerm::renameVars(RenameMap &names) const {
    UTermVec args;
    for (auto &x : this->args) { args.emplace_back(x->renameVars(names)); }
    return make_locatable<FunctionTerm>(loc(), name, std::move(args));
}

// }}}
// {{{ definition of Term::gterm

SGRef Term::_newRef(RenameMap &names, Term::ReferenceMap &refs) const {
    UTerm x(renameVars(names));
    auto &ref = refs[x.get()];
    if (!ref) { ref = std::make_shared<GRef>(std::move(x)); }
    return ref;
}

UGTerm PoolTerm::gterm(RenameMap &names, ReferenceMap &refs) const   { return make_unique<GVarTerm>(_newRef(names, refs)); }
UGTerm ValTerm::gterm(RenameMap &, ReferenceMap &) const    { return make_unique<GValTerm>(value); }
UGTerm LinearTerm::gterm(RenameMap &names, ReferenceMap &refs) const { return make_unique<GLinearTerm>(var->_newRef(names, refs), m, n); }
UGTerm VarTerm::gterm(RenameMap &names, ReferenceMap &refs) const    { return make_unique<GVarTerm>(_newRef(names, refs)); }
UGTerm UnOpTerm::gterm(RenameMap &names, ReferenceMap &refs) const   { return make_unique<GVarTerm>(_newRef(names, refs)); }
UGTerm BinOpTerm::gterm(RenameMap &names, ReferenceMap &refs) const  { return make_unique<GVarTerm>(_newRef(names, refs)); }
UGTerm DotsTerm::gterm(RenameMap &names, ReferenceMap &refs) const   { return make_unique<GVarTerm>(_newRef(names, refs)); }
UGTerm LuaTerm::gterm(RenameMap &names, ReferenceMap &refs) const    { return make_unique<GVarTerm>(_newRef(names, refs)); }
UGTerm FunctionTerm::gterm(RenameMap &names, ReferenceMap &refs) const {
    UGTermVec args;
    for (auto &x : this->args) { args.emplace_back(x->gterm(names, refs)); }
    return make_unique<GFunctionTerm>(name, std::move(args));
}
UGTerm Term::gterm() const {
    RenameMap names;
    ReferenceMap refs;
    return gterm(names, refs);
}

// }}}
// {{{ definition of Term::collectIds

void PoolTerm::collectIds(VarSet &x) const {
    for (auto &y : args) { y->collectIds(x); }
}
void ValTerm::collectIds(VarSet &x) const {
    if (value.type() == Value::ID) { x.emplace(value.string()); }
}
void LinearTerm::collectIds(VarSet &) const {
}
void VarTerm::collectIds(VarSet &) const {
}
void UnOpTerm::collectIds(VarSet &x) const {
    arg->collectIds(x);
}
void BinOpTerm::collectIds(VarSet &x) const {
    left->collectIds(x);
    right->collectIds(x);
}
void DotsTerm::collectIds(VarSet &x) const {
    left->collectIds(x);
    right->collectIds(x);
}
void LuaTerm::collectIds(VarSet &x) const {
    for (auto &y : args) { y->collectIds(x); }
}
void FunctionTerm::collectIds(VarSet &x) const {
    for (auto &y : args) { y->collectIds(x); }
}

// }}}
// {{{ definition of Term::replace

UTerm PoolTerm::replace(Defines &x, bool replace) {
    for (auto &y : args) { Term::replace(y, y->replace(x, replace)); }
    return nullptr;
}
UTerm ValTerm::replace(Defines &x, bool replace) {
    Value retVal;
    UTerm retTerm;
    x.apply(value, retVal, retTerm, replace);
    if (retVal.type() != Value::SPECIAL) { value = retVal; }
    else                                 { return std::move(retTerm); }
    return nullptr;
}
UTerm LinearTerm::replace(Defines &, bool) {
    return nullptr;
}
UTerm VarTerm::replace(Defines &, bool) {
    return nullptr;
}
UTerm UnOpTerm::replace(Defines &x, bool) {
    Term::replace(arg, arg->replace(x, true));
    return nullptr;
}
UTerm BinOpTerm::replace(Defines &x, bool) {
    Term::replace(left, left->replace(x, true));
    Term::replace(right, right->replace(x, true));
    return nullptr;
}
UTerm DotsTerm::replace(Defines &x, bool) {
    Term::replace(left, left->replace(x, true));
    Term::replace(right, right->replace(x, true));
    return nullptr;
}
UTerm LuaTerm::replace(Defines &x, bool) {
    for (auto &y : args) { Term::replace(y, y->replace(x, true)); }
    return nullptr;
}
UTerm FunctionTerm::replace(Defines &x, bool) {
    for (auto &y : args) { Term::replace(y, y->replace(x, true)); }
    return nullptr;
}

// }}}
// {{{ definition of Term::estimate

double PoolTerm::estimate(double, VarSet const &) const {
    return 0;
}
double ValTerm::estimate(double, VarSet const &) const {
    return 0;
}
double LinearTerm::estimate(double size, VarSet const &bound) const {
    return var->estimate(size, bound);
}
double VarTerm::estimate(double size, VarSet const &bound) const {
    return bound.find(name) == bound.end() ? size : 0.0;
}
double UnOpTerm::estimate(double, VarSet const &) const {
    return 0;
}
double BinOpTerm::estimate(double, VarSet const &) const {
    return 0;
}
double DotsTerm::estimate(double, VarSet const &) const {
    return 0;
}
double LuaTerm::estimate(double, VarSet const &) const {
    return 0;
}
double FunctionTerm::estimate(double size, VarSet const &bound) const {
    double ret = 0.0;
    if (!args.empty()) {
        double root = std::max(1.0, std::pow(((*name).empty() ? size : size/2.0), 1.0/args.size()));
        for (auto &x : args) { ret += x->estimate(root, bound); }
        ret /= args.size();
    }
    return ret;
}

// }}}
// {{{ definition of Term::isEDB

Value PoolTerm::isEDB() const { return {}; }
Value ValTerm::isEDB() const { return value; }
Value LinearTerm::isEDB() const { return {}; }
Value VarTerm::isEDB() const { return {}; }
Value UnOpTerm::isEDB() const { return {}; }
Value BinOpTerm::isEDB() const { return {}; }
Value DotsTerm::isEDB() const { return {}; }
Value LuaTerm::isEDB() const { return {}; }
Value FunctionTerm::isEDB() const {
    cache.clear();
    for (auto &x : args) { 
        cache.emplace_back(x->isEDB());
        if (cache.back().type() == Value::SPECIAL) { return {}; }
    }
    return Value(name, cache);
}

// }}}

// {{{ definition of operator<< for BinOp and UnOp

int eval(UnOp op, int x) {
    switch (op) {
        case UnOp::NEG: { return -x; }
        case UnOp::ABS: { return std::abs(x); }
        case UnOp::NOT: { return ~x; }
    }
    assert(false);
    return 0;
}
std::ostream &operator<<(std::ostream &out, UnOp op) {
    switch (op) {
        case UnOp::ABS: { out << "#abs"; break; }
        case UnOp::NOT: { out << "~"; break; }
        case UnOp::NEG: { out << "-"; break; }
    }
    return out;
}

int eval(BinOp op, int x, int y) {
    switch (op) {
        case BinOp::XOR: { return x ^ y; }
        case BinOp::OR:  { return x | y; }
        case BinOp::AND: { return x & y; }
        case BinOp::ADD: { return x + y; }
        case BinOp::SUB: { return x - y; }
        case BinOp::MUL: { return x * y; }
        case BinOp::MOD: { return x % y; }
        case BinOp::POW: { return ipow(x, y); }
        case BinOp::DIV: { 
            assert(y != 0 && "must be checked before call");
            return x / y;
        }
    }
    assert(false);
    return 0;
}
int toNum(UTerm const &x) {
    Value y(x->eval());
	if (y.type() == Value::NUM) { return y.num(); }
	else {
		GRINGO_REPORT(W_TERM_UNDEFINED)
			<< x->eval() << ": warning: expected a number, a zero is substituted:\n"
			<< "  " << *x << "\n";
		return 0;
	}
}
std::ostream &operator<<(std::ostream &out, BinOp op) {
    switch (op) {
        case BinOp::AND: { out << "&"; break; }
        case BinOp::OR:  { out << "?"; break; }
        case BinOp::XOR: { out << "^"; break; }
        case BinOp::POW: { out << "**"; break; }
        case BinOp::ADD: { out << "+"; break; }
        case BinOp::SUB: { out << "-"; break; }
        case BinOp::MUL: { out << "*"; break; }
        case BinOp::DIV: { out << "/"; break; }
        case BinOp::MOD: { out << "\\"; break; }
    }
    return out;
}

// }}}
// {{{ definition of Term and auxiliary functions

void Term::collect(VarTermSet &x) const {
    VarTermBoundVec vars;
    collect(vars, false);
    for (auto &y : vars) { x.emplace(*y.first); }
}

UTermVec unpool(UTerm const &x) {
    UTermVec pool;
    x->unpool(pool);
    return pool;
}

bool Term::isZero() const {
    return getInvertibility() == Term::CONSTANT && eval() == Value(0);
}

FWString Term::uniqueName(unsigned &auxNum, char const *prefix) {
    return FWString(prefix + std::to_string(auxNum++));
}
UTerm Term::uniqueVar(Location const &loc, unsigned &auxNum, unsigned level, const char *prefix) {
    return make_locatable<VarTerm>(loc, uniqueName(auxNum, prefix), std::make_shared<Value>(), level);
}
UTerm Term::uniqueVal(Location const &loc, unsigned &auxNum, const char *prefix) {
    return make_locatable<ValTerm>(loc, Value(uniqueName(auxNum, prefix)));
}

UTerm Term::insert(ArithmeticsMap &arith, unsigned &auxNum, UTerm &&term) {
    unsigned level = term->getLevel();
    assert(level < arith.size());
    auto ret = arith[level].emplace(std::move(term), nullptr);
    if (ret.second) { ret.first->second = uniqueVar(ret.first->first->loc(), auxNum, level, "#Arith"); }
    return get_clone(ret.first->second);
}

// }}}
// {{{ definition of PoolTerm

PoolTerm::PoolTerm(UTermVec &&terms)
    : args(std::move(terms)) { }

PoolTerm::~PoolTerm() { }

// }}}
// {{{ definition of ValTerm

ValTerm::ValTerm(Value value)
    : value(value) { }

ValTerm::~ValTerm() { }

// }}}
// {{{ definition of VarTerm

VarTerm::VarTerm(FWString name, SVal ref, unsigned level, bool bindRef)
    : name(name) 
    , ref(ref)
    , bindRef(bindRef)
    , level(level) { assert(ref || *name == "_"); }

VarTerm::~VarTerm() { }

// }}}
// {{{ definition of LinearTerm

LinearTerm::LinearTerm(VarTerm const &var, int m, int n) 
    : var(static_cast<VarTerm*>(var.clone()))
    , m(m) 
    , n(n) { }

LinearTerm::LinearTerm(UVarTerm &&var, int m, int n) 
    : var(std::move(var))
    , m(m) 
    , n(n) { }

std::unique_ptr<LinearTerm> LinearTerm::create(Location const &loc, ScriptMap &scripts, unsigned &auxNum, FWString name, UTermVec &&args) {
    scripts.emplace_back(Term::uniqueVar(loc, auxNum, 0, "#Script"), name, std::move(args));
    return make_locatable<LinearTerm>(loc, static_cast<VarTerm&>(*std::get<0>(scripts.back())), 1, 0);
}

std::unique_ptr<LinearTerm> LinearTerm::create(Location const &loc, DotsMap &dots, unsigned &auxNum, UTerm &&left, UTerm &&right) {
    dots.emplace_back(Term::uniqueVar(loc, auxNum, 0, "#Range"), std::move(left), std::move(right));
    return make_locatable<LinearTerm>(loc, static_cast<VarTerm&>(*std::get<0>(dots.back())), 1, 0);
}

LinearTerm::~LinearTerm() { }

// }}}
// {{{ definition of UnOpTerm

UnOpTerm::UnOpTerm(UnOp op, UTerm &&arg)
    : op(op)
    , arg(std::move(arg)) { }

UnOpTerm::~UnOpTerm() { }

// }}}
// {{{ definition of BinOpTerm

BinOpTerm::BinOpTerm(BinOp op, UTerm &&left, UTerm &&right)
    : op(op)
    , left(std::move(left))
    , right(std::move(right)) { }

BinOpTerm::~BinOpTerm() { }

// }}}
// {{{ definition of DotsTerm

DotsTerm::DotsTerm(UTerm &&left, UTerm &&right)
    : left(std::move(left))
    , right(std::move(right)) { }

DotsTerm::~DotsTerm() { }

// }}}
// {{{ definition of LuaTerm

LuaTerm::LuaTerm(FWString name, UTermVec &&args)
    : name(name)
    , args(std::move(args)) { }

LuaTerm::~LuaTerm() { }

// }}}
// {{{ definition of FunctionTerm

FunctionTerm::FunctionTerm(FWString name, UTermVec &&args)
    : name(name)
    , args(std::move(args)) { }

FunctionTerm::~FunctionTerm() { }

// }}}

} // namespace Gringo
