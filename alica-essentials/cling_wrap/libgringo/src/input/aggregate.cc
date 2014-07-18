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

#include "gringo/bug.hh"
#include "gringo/logger.hh"
#include "gringo/input/aggregate.hh"

namespace Gringo { namespace Input {

// {{{ definition of AssignLevel

void AssignLevel::add(VarTermBoundVec &vars) {
    for (auto &occ : vars) { occurr[occ.first->name].emplace_back(occ.first); }
}
AssignLevel &AssignLevel::subLevel() {
    childs.emplace_front();
    return childs.front();
}
void AssignLevel::assignLevels() {
    BoundSet bound;
    assignLevels(0, bound);
}
void AssignLevel::assignLevels(unsigned level, BoundSet const &parent) {
    BoundSet bound(parent);
    for (auto &occs : occurr) { 
        auto ret = bound.emplace(occs.first, level);
        for (auto &occ : occs.second) { occ->level = ret.first->second; }
    }
    for (auto &child : childs) { child.assignLevels(level + 1, bound); }
}
AssignLevel::~AssignLevel() { }

// }}}
// {{{ definition of CheckLevel

bool CheckLevel::Ent::operator<(Ent const &) const { return false; }
CheckLevel::CheckLevel(Location const &loc, Printable const &p) : loc(loc), p(p) { }
CheckLevel::CheckLevel(CheckLevel &&) = default;
CheckLevel::SC::VarNode &CheckLevel::var(VarTerm &var) {
    auto &node = vars[var.name];
    if (!node) { node = &dep.insertVar(&var); }
    return *node;
}
bool CheckLevel::check() {
    dep.order();
    auto vars(dep.open());
    if (!vars.empty()) {
        auto cmp = [](SC::VarNode const *x, SC::VarNode const *y) -> bool{ 
            if (x->data->name != y->data->name) { return *x->data->name < *y->data->name; }
            return x->data->loc() < y->data->loc();
        };
        std::sort(vars.begin(), vars.end(), cmp);
        std::ostringstream msg;
        msg << loc << ": error: unsafe variables in\n  " << p << "\n";
        for (auto &x : vars) { msg << x->data->loc() << ": note: '" << x->data->name << "' is unsafe\n"; }
        GRINGO_REPORT(ERROR) << msg.str();
    }
    return vars.empty();
}
CheckLevel::~CheckLevel() { }

// }}}
// {{{ declaration of ToGroundArg

ToGroundArg::ToGroundArg(unsigned &auxNames, PredDomMap &domains) 
    : auxNames(auxNames)
    , domains(domains) { }
FWString ToGroundArg::newId(bool increment) { 
    auxNames+= increment;
    return "#d" + std::to_string(auxNames-increment);
}
UTermVec ToGroundArg::getGlobal(VarTermBoundVec const &vars) {
    std::unordered_set<FWString> seen;
    UTermVec global;
    for (auto &occ : vars) {
        if (occ.first->level == 0 && seen.emplace(occ.first->name).second) {
            global.emplace_back(occ.first->clone());
        }
    }
    return global;
}
UTerm ToGroundArg::newId(UTermVec &&global, Location const &loc, bool increment) {
    if (!global.empty()) { return make_locatable<FunctionTerm>(loc, newId(increment), std::move(global)); }
    else                 { return make_locatable<ValTerm>(loc, newId(increment)); }
}
ToGroundArg::~ToGroundArg() { }

// }}}

} } // namespace Input Gringo
