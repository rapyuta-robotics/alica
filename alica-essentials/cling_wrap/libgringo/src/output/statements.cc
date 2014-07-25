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

#include <gringo/output/statements.hh>
#include <gringo/output/lparseoutputter.hh>

namespace Gringo { namespace Output {

// {{{ definition of helpers to print interval sets

namespace Debug {

std::ostream &operator<<(std::ostream &out, IntervalSet<Value>::LBound const &x) {
    out << (x.inclusive ? "[" : "(") << x.bound;
    return out;
}

std::ostream &operator<<(std::ostream &out, IntervalSet<Value>::RBound const &x) {
    out << x.bound << (x.inclusive ? "]" : ")");
    return out;
}

std::ostream &operator<<(std::ostream &out, IntervalSet<Value>::Interval const &x) {
    out << x.left << "," << x.right;
    return out;
}

std::ostream &operator<<(std::ostream &out, IntervalSet<Value> const &x) {
    out << "{";
    print_comma(out, x.vec, ",", [](std::ostream &out, IntervalSet<Value>::Interval const &x) { out << x; });
    out << "}";
    return out;
}

} // namespace

// }}}

// {{{ definition of Rule

Rule::Rule() { }
Rule::Rule(PredicateDomain::element_type *head, ULitVec &&body) : head(head), body(std::move(body)) { }
void Rule::printPlain(std::ostream &out) const {
    bool isShow = head && head->first.sig() == Signature("#show", 2);
    if (isShow)    { out << "#show " << ((*(head->first.args().begin()+1)).num() == 1 ? "$" : "") << *head->first.args().begin(); }
    else if (head) { out << head->first; }
    if (!body.empty() || !head) { out << (isShow ? ":" : ":-"); }
    if (!body.empty()) {
        auto it(body.begin()), ie(body.end());
        it->get()->printPlain(out);
        for (++it; it != ie; ++it) { out << ";"; it->get()->printPlain(out); }
    }
    out << ".\n";
}
void Rule::toLparse(LparseTranslator &x) {
    if (!head) {
        Value value;
        auto isBound = [&]() {
            if (body.empty())     { return false; }
            for (auto &x : body) {
                if (!x->isBound(value, true)) { return false; }
            }
            return true;
        };
        if (isBound()) {
            std::vector<CSPBound> bounds;
            for (auto &y : body) {
                bounds.emplace_back(std::numeric_limits<int>::min(), std::numeric_limits<int>::max()-1);
                y->updateBound(bounds.back(), true);
            }
            x.addBounds(value, bounds);
            return;
        }
    }
    for (auto &y : body) { Term::replace(y, y->toLparse(x)); }
    x(*this);
}
void Rule::printLparse(LparseOutputter &out) const {
    unsigned headUid;
    if (!head)                       { headUid = out.falseUid(); }
    else if (!head->second.hasUid()) { headUid = out.newUid(); head->second.uid(headUid); }
    else                             { headUid = head->second.uid(); }
    LparseOutputter::LitVec lits;
    for (auto &x : body) { lits.emplace_back(x->lparseUid(out)); }
    out.printBasicRule(headUid, lits);
}
Rule *Rule::clone() const {
    auto ret(make_unique<Rule>());
    ret->body = get_clone(body);
    ret->head = head;
    return ret.release();
}
bool Rule::isIncomplete() const {
    for (auto &x : body) {
        if (x->isIncomplete()) {
            return true;
        }
    }
    return false;
}
Rule::~Rule() { }

void RuleRef::printPlain(std::ostream &out) const {
    bool isShow = head && head->first.sig() == Signature("#show", 2);
    if (isShow)    { out << "#show " << ((*(head->first.args().begin()+1)).num() == 1 ? "$" : "") << *head->first.args().begin(); }
    else if (head) { out << head->first; }
    if (!body.empty() || !head) { out << (isShow ? ":" : ":-"); }
    if (!body.empty()) {
        auto it(body.begin()), ie(body.end());
        it->get().printPlain(out);
        for (++it; it != ie; ++it) { out << ";"; it->get().printPlain(out); }
    }
    out << ".\n";
}
Rule *RuleRef::clone() const {
    auto ret(make_unique<Rule>());
    for (Literal &x : body) { ret->body.emplace_back(ULit(x.clone())); }
    ret->head = head;
    return ret.release();
}
void RuleRef::toLparse(LparseTranslator &x) {
    ULitVec bd;
    for (auto &y : body) {
        // TODO: stupid copy and paste
        if (!head) {
            Value value;
            auto isBound = [&]() {
                if (body.empty())     { return false; }
                for (auto &x : body) {
                    if (!x.get().isBound(value, true)) { return false; }
                }
                return true;
            };
            if (isBound()) {
                std::vector<CSPBound> bounds;
                for (auto &y : body) {
                    bounds.emplace_back(std::numeric_limits<int>::min(), std::numeric_limits<int>::max()-1);
                    y.get().updateBound(bounds.back(), true);
                }
                x.addBounds(value, bounds);
                return;
            }
        }
        if (ULit z = y.get().toLparse(x)) {
            bd.emplace_back(std::move(z));
            y = *bd.back();
        }
    }
    x(*this);
}
void RuleRef::printLparse(LparseOutputter &out) const {
    unsigned headUid;
    if (!head)                       { headUid = out.falseUid(); }
    else if (!head->second.hasUid()) { headUid = out.newUid(); head->second.uid(headUid); }
    else                             { headUid = head->second.uid(); }
    LparseOutputter::LitVec lits;
    for (Literal &x : body) { lits.emplace_back(x.lparseUid(out)); }
    out.printBasicRule(headUid, lits);
}
bool RuleRef::isIncomplete() const {
    for (Literal &x : body) {
        if (x.isIncomplete()) {
            return true;
        }
    }
    return false;
}
RuleRef::~RuleRef() { }

// }}}
// {{{ definition of LparseRule

LparseRule::LparseRule(ULitVec &&body) : LparseRule(HeadVec(), std::move(body), false) { }
LparseRule::LparseRule(ULit &&b1, ULit &&b2) : LparseRule(ULitVec()) {
    if (b1) { body.emplace_back(std::move(b1)); }
    if (b2) { body.emplace_back(std::move(b2)); }
}
LparseRule::LparseRule(SAuxAtom head, ULit &&b1, ULit &&b2) : LparseRule(head, ULitVec()) {
    if (b1) { body.emplace_back(std::move(b1)); }
    if (b2) { body.emplace_back(std::move(b2)); }
}
LparseRule::LparseRule(PredicateDomain::element_type &head, ULit &&b1, ULit &&b2) : LparseRule(head, ULitVec()) {
    if (b1) { body.emplace_back(std::move(b1)); }
    if (b2) { body.emplace_back(std::move(b2)); }
}
LparseRule::LparseRule(SAuxAtom head, ULitVec &&body) : choice(false), body(std::move(body)) {
    assert(head);
    auxHead.emplace_back(head);
}
LparseRule::LparseRule(PredicateDomain::element_type &head, ULitVec &&body) : choice(false), body(std::move(body)) {
    this->head.emplace_back(head);
}
LparseRule::LparseRule(SAuxAtomVec &&head, ULitVec &&body, bool choice) : choice(choice), auxHead(std::move(head)), body(std::move(body)) { }
LparseRule::LparseRule(HeadVec &&head, ULitVec &&body, bool choice) : choice(choice), head(std::move(head)), body(std::move(body)) { }
LparseRule::LparseRule(HeadVec &&head, SAuxAtomVec &&auxHead, ULitVec &&body, bool choice) : choice(choice), head(std::move(head)), auxHead(std::move(auxHead)), body(std::move(body)) { }
void LparseRule::printPlain(std::ostream &out) const {
    if (choice) { out << "{"; }
    print_comma(out, head, choice ? ";" : "|", [](std::ostream &out, PredicateDomain::element_type const &x) { out << x.first;  });
    if (!head.empty() && !auxHead.empty()) { out << (choice ? ";" : "|"); }
    print_comma(out, auxHead, "|", [](std::ostream &out, SAuxAtom const &x) { out << *x; });
    if (choice) { out << "}"; }
    if (!body.empty()) {
        out << ":-";
        auto it(body.begin()), ie(body.end());
        it->get()->printPlain(out);
        for (++it; it != ie; ++it) { out << ";"; it->get()->printPlain(out); }
    }
    out << ".\n";
}
void LparseRule::toLparse(LparseTranslator &x) {
    // TODO: stupid copy and paste
    if (head.empty() && auxHead.empty()) {
        Value value;
        auto isBound = [&]() {
            if (body.empty())     { return false; }
            for (auto &x : body) {
                if (!x->isBound(value, true)) { return false; }
            }
            return true;
        };
        if (isBound()) {
            std::vector<CSPBound> bounds;
            for (auto &y : body) {
                bounds.emplace_back(std::numeric_limits<int>::min(), std::numeric_limits<int>::max()-1);
                y->updateBound(bounds.back(), true);
            }
            x.addBounds(value, bounds);
            return;
        }
    }
    for (auto &y : body) { Term::replace(y, y->toLparse(x)); }
    x(*this);
}
void LparseRule::printLparse(LparseOutputter &out) const {
    LparseOutputter::AtomVec atoms;
    for (PredicateDomain::element_type &x : head) {
        if (!x.second.hasUid()) { x.second.uid(out.newUid()); }
        atoms.emplace_back(x.second.uid());
    }
    for (auto &x : auxHead) {
        if (!x->uid) { x->uid = out.newUid(); }
        atoms.emplace_back(x->uid);
    }
    LparseOutputter::LitVec lits;
    for (auto &x : body) { lits.emplace_back(x->lparseUid(out)); }

    if (atoms.empty())          { out.printBasicRule(out.falseUid(), lits); }
    else if (choice)            { out.printChoiceRule(atoms, lits); }
    else if (atoms.size() == 1) { out.printBasicRule(atoms.front(), lits); }
    else                        { out.printDisjunctiveRule(atoms, lits); }
}
LparseRule *LparseRule::clone() const {
    auto ret(make_unique<LparseRule>(get_clone(head), get_clone(auxHead), get_clone(body), choice));
    return ret.release();
}
bool LparseRule::isIncomplete() const { return false; }
LparseRule::~LparseRule() { }

// }}}
// {{{ definition of WeightRule

WeightRule::WeightRule(SAuxAtom head, unsigned lower, ULitBoundVec &&body) : head(std::move(head)), body(std::move(body)), lower(lower) { }
void WeightRule::printPlain(std::ostream &out) const {
    out << *head << ":-" << lower << "{";
    if (!body.empty()) {
        auto it(body.begin()), ie(body.end());
        it->first->printPlain(out);
        out << "=" << it->second;
        for (++it; it != ie; ++it) { 
            out << ",";
            it->first->printPlain(out);
            out << "=" << it->second;
        }
    }
    out << "}.\n";
}
void WeightRule::toLparse(LparseTranslator &x) {
    for (auto &y : body) { Term::replace(y.first, y.first->toLparse(x)); }
    x(*this);
}
void WeightRule::printLparse(LparseOutputter &out) const {
    if(!head->uid) { head->uid = out.newUid(); }
    bool card = true;
    for (auto &x : body) {
        if (x.second != 1) { 
            card = false; 
            break;
        }
    }
    if (card) {
        LparseOutputter::LitVec lits;
        for (auto &x : body) { lits.emplace_back(x.first->lparseUid(out)); }
        out.printCardinalityRule(head->uid, lower, lits);
    }
    else {
        LparseOutputter::LitWeightVec lits;
        for (auto &x : body) { lits.emplace_back(x.first->lparseUid(out), x.second); }
        out.printWeightRule(head->uid, lower, lits);
    }
}
WeightRule *WeightRule::clone() const {
    auto ret(make_unique<WeightRule>(head, lower, get_clone(body)));
    return ret.release();
}
bool WeightRule::isIncomplete() const { return false; }
WeightRule::~WeightRule() { }

// }}}
// {{{ definition of HeadAggregateState

HeadAggregateElement::Cond::Cond(PredicateDomain::element_type *head, unsigned headNum, Output::ULitVec &&lits)
    : head(head)
    , headNum(headNum)
    , lits(std::move(lits)) { }

HeadAggregateState::HeadAggregateState() { throw std::runtime_error("must not happen"); }
HeadAggregateState::HeadAggregateState(AggregateFunction fun, unsigned generation) 
    : _generation(generation) {
    switch (fun) {
        case AggregateFunction::MIN: {
            valMin = Value(false);
            valMax = Value(false);
            break;
        }
        case AggregateFunction::MAX: {
            valMin = Value(true);
            valMax = Value(true);
            break;
        }
        default: {
            intMin = 0;
            intMax = 0;
            break;
        }
    }
}
void HeadAggregateState::accumulate(ValVec const &tuple, AggregateFunction fun, PredicateDomain::element_type *head, unsigned headNum, LitVec const &lits, Location const &loc) {
    bool ignore = neutral(tuple, fun, loc);
    if (!ignore || head) {
        bool fact(lits.empty() && (!head || (head->second.defined() && head->second.fact(false))));
        auto ret(elems.emplace_back(std::piecewise_construct, std::forward_as_tuple(tuple), std::forward_as_tuple()));
        bool remove(fact && !ret.first->second.fact && !ret.second);
        if (ret.second || remove || head) {
            auto &elem(ret.first->second);
            elem.fact = elem.fact || fact;
            Output::ULitVec uLits;
            for (Output::Literal &x : lits) { uLits.emplace_back(x.clone()); }
            elem.conds.emplace_back(head, headNum, std::move(uLits));
            if ((ret.second || remove) && !ignore) {
                switch (fun) {
                    case AggregateFunction::MIN: {
                        Value val = tuple.front();
                        if (fact) { valMax = std::min<Value>(valMax, val); }
                        valMin = std::min<Value>(valMin, val);
                        break;
                    }
                    case AggregateFunction::MAX: {
                        Value val = tuple.front();
                        if (fact) { valMin = std::max<Value>(valMin, val); }
                        valMax = std::max<Value>(valMax, val);
                        break;
                    }
                    default: {
                        int val = fun == AggregateFunction::COUNT ? 1 : tuple.front().num();
                        if (fact) {
                            if (remove) {
                                if (val < 0) { intMax+= val; }
                                else         { intMin+= val; }
                            }
                            else {
                                intMin+= val;
                                intMax+= val;
                            }
                        }
                        else {
                            if (val < 0) { intMin+= val; }
                            else         { intMax+= val; }
                        }
                        break;
                    }
                }
            }
        }
    }
}
bool HeadAggregateState::defined() const { return true; }
HeadAggregateState::Bounds::Interval HeadAggregateState::range(AggregateFunction fun) const {
    if (fun != AggregateFunction::MIN && fun != AggregateFunction::MAX) {
        return {{clamp(intMin), true}, {clamp(intMax), true}};
    }
    else { return {{valMin, true}, {valMax, true}}; }
}
unsigned HeadAggregateState::generation() const { return _generation; }
bool HeadAggregateState::fact(bool) const { return false; }
HeadAggregateState::element_type &HeadAggregateState::ignore() { throw std::logic_error("HeadAggregateState::ignore must not be called"); }

// }}}
// {{{ definition of HeadAggregateRule

void HeadAggregateRule::toLparse(LparseTranslator &x) {
    // TODO: this algorithm suffers from severe uglyness!!!
    // TODO: handle empty bounds here...
    auto rng(repr->range(fun));
    if (rng.empty()) {
        LparseRule(std::move(body)).toLparse(x);
        return;
    }
    SAuxAtom b;
    if (!body.empty()) {
        // b :- body.
        b = std::make_shared<AuxAtom>(x.auxAtom());
        LparseRule(b, std::move(body)).toLparse(x);
    }
    using GroupByCond = unique_list<
        std::pair<std::reference_wrapper<ULitVec>, std::pair<std::shared_ptr<AuxAtom>, std::vector<std::pair<FWValVec, HeadAggregateElement::Cond&>>>>, 
        extract_first<ULitVec>, 
        value_hash<ULitVec>, 
        value_equal_to<ULitVec>
    >;
    GroupByCond groupByCond;
    for (auto &y : repr->elems) {
        for (auto &z : y.second.conds) {
            auto ret(groupByCond.emplace_back(std::piecewise_construct, std::forward_as_tuple(z.lits), std::forward_as_tuple()));
            ret.first->second.second.emplace_back(y.first, z);
        }
    }
    for (auto &y : groupByCond) {
        if (!y.first.get().empty()) { 
            // c :- C.
            y.second.first = std::make_shared<AuxAtom>(x.auxAtom());
            LparseRule(y.second.first, get_clone(y.first.get())).toLparse(x);
        }
        // { heads } :- c, b.
        LparseRule::HeadVec atoms;
        for (auto &z : y.second.second) {
            if (z.second.head) { atoms.emplace_back(*z.second.head); }
        }
        if (!atoms.empty()) {
            ULitVec lits;
            if (y.second.first) { lits.emplace_back(make_unique<AuxLiteral>(y.second.first, false)); }
            if (b)              { lits.emplace_back(make_unique<AuxLiteral>(b, false)); }
            LparseRule(std::move(atoms), std::move(lits), true).toLparse(x);
        }
    }
    if (!repr->bounds.contains(rng)) {
        using TupleAtoms = std::unordered_map<FWValVec, SAuxAtom>;
        TupleAtoms tupleAtoms;
        using ULitValVec = std::vector<std::pair<ULit,Value>>;
        ULitValVec elemVec;
        for (auto &y : repr->elems) { 
            Value weight(getWeight(fun, y.first));
            if (y.second.conds.size() != 1 || !y.second.conds.front().head || !y.second.conds.front().lits.empty()) {
                SAuxAtom atom(std::make_shared<AuxAtom>(x.auxAtom()));
                tupleAtoms.emplace(y.first, atom);
                elemVec.emplace_back(make_unique<AuxLiteral>(atom, false), weight);
            }
            else {
                elemVec.emplace_back(make_unique<PredicateLiteral>(NAF::POS, *y.second.conds.front().head), weight);
            }
        }
        // t :- c, h.
        for (auto &y : groupByCond) {
            for (auto &z : y.second.second) {
                auto it(tupleAtoms.find(z.first));
                if (it != tupleAtoms.end()) {
                    LparseRule(
                        it->second, 
                        y.second.first ? make_unique<AuxLiteral>(y.second.first, false) : nullptr, 
                        z.second.head  ? make_unique<PredicateLiteral>(NAF::POS, *z.second.head) : nullptr).toLparse(x);
                }
            }
        }
        switch (fun) {
            case AggregateFunction::COUNT:
            case AggregateFunction::SUMP:
            case AggregateFunction::SUM: {
                int  shift = 0;
                WeightRule::ULitBoundVec elemPos;
                for (auto &y : elemVec) {
                    int weight = y.second.num();
                    if (weight < 0) {
                        shift+= -weight;
                        SAuxAtom neg(std::make_shared<AuxAtom>(x.auxAtom()));
                        LparseRule(neg, get_clone(y.first), nullptr).toLparse(x);
                        elemPos.emplace_back(make_unique<AuxLiteral>(neg, true), -weight);
                    }
                    else { elemPos.emplace_back(get_clone(y.first), weight); }
                }
                
                SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                for (auto &y : repr->bounds) {
                    ULitVec aLits;
                    if (rng.left < y.left) {
                        // a :- L { }.
                        SAuxAtom a(std::make_shared<AuxAtom>(x.auxAtom()));
                        WeightRule wr(a, std::max(0, toInt(y.left) + shift), get_clone(elemPos));
                        wr.toLparse(x);
                        aLits.emplace_back(make_unique<AuxLiteral>(a, false));
                    }
                    if (y.right < rng.right) {
                        // a :- U+1 { }.
                        SAuxAtom a(std::make_shared<AuxAtom>(x.auxAtom()));
                        WeightRule wr(a, std::max(0, toInt(y.right) + 1 + shift), get_clone(elemPos));
                        wr.toLparse(x);
                        aLits.emplace_back(make_unique<AuxLiteral>(a, true));
                    } 
                    LparseRule(aggr, std::move(aLits)).toLparse(x);
                }
                // :- b, not aggr.
                LparseRule(b ? make_unique<AuxLiteral>(b, false) : nullptr, make_unique<AuxLiteral>(aggr, true)).toLparse(x);
                break;
            }
            case AggregateFunction::MIN:
            case AggregateFunction::MAX: {
                SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                for (auto &y : repr->bounds) {
                    ULitVec aLits;
                    using namespace Debug;
                    if (fun == AggregateFunction::MIN ? y.right < rng.right : rng.left < y.left) {
                        SAuxAtom a(std::make_shared<AuxAtom>(x.auxAtom()));
                        for (auto &z : elemVec) {
                            // a :- 1 { between }
                            if (y.contains(z.second)) { LparseRule(a, get_clone(z.first), nullptr).toLparse(x); }
                        }
                        aLits.emplace_back(make_unique<AuxLiteral>(a, false));
                    }
                    if (fun == AggregateFunction::MIN ? rng.left < y.left : y.right < rng.right) {
                        SAuxAtom a(std::make_shared<AuxAtom>(x.auxAtom()));
                        for (auto &z : elemVec) {
                            if (fun == AggregateFunction::MIN ? z.second < y : y < z.second) {
                                // if min:  a :- 1 { below }
                                // eif max: a :- 1 { above }
                                LparseRule(a, get_clone(z.first), nullptr).toLparse(x);
                            }
                        }
                        aLits.emplace_back(make_unique<AuxLiteral>(a, true));
                    }
                    LparseRule(aggr, std::move(aLits)).toLparse(x);
                }
                LparseRule(b ? make_unique<AuxLiteral>(b, false) : nullptr, make_unique<AuxLiteral>(aggr, true)).toLparse(x);
                break;
            }
        }
    }
}
void HeadAggregateRule::printLparse(LparseOutputter &) const {
    throw std::runtime_error("HeadAggregateRule::printLparse must be called after HeadAggregateRule::toLparse");
}
void HeadAggregateRule::printElem(std::ostream &out, HeadAggregateState::ElemSet::value_type const &x) {
    bool comma = false;
    for (auto &y : x.second.conds) {
        if (comma) { out << ";"; }
        else       { comma = true; }
        print_comma(out, x.first, ",");
        out << ":";
        if (y.head) { out << y.head->first; }
        else        { out << "#true"; }
        if (!y.lits.empty()) {
            out << ":";
            using namespace std::placeholders;
            print_comma(out, y.lits, ",", std::bind(&Literal::printPlain, _2, _1));
        }
    }
}
void HeadAggregateRule::printPlain(std::ostream &out) const {
    auto it = bounds.begin(), ie = bounds.end();
    if (it != ie) { out << it->second << inv(it->first); ++it; }
    out << fun << "{";
    print_comma(out, repr->elems, ";", &HeadAggregateRule::printElem);
    out << "}";
    for (; it != ie; ++it) { out << it->first << it->second; }
    if (!body.empty()) {
        out << ":-";
        using namespace std::placeholders;
        print_comma(out, body, ";", std::bind(&Literal::printPlain, _2, _1));
    }
    out << ".\n";
}
bool HeadAggregateRule::isIncomplete() const { return true; }
HeadAggregateRule *HeadAggregateRule::clone() const { 
    auto ret(make_unique<HeadAggregateRule>());
    ret->body   = get_clone(body);
    ret->bounds = bounds;
    ret->repr   = repr;
    ret->fun    = fun;
    return ret.release();
}
HeadAggregateRule::~HeadAggregateRule() { }

// }}}
// {{{ definition of DisjunctionState

DisjunctionState::DisjunctionState() { throw std::runtime_error("must not happen"); }
DisjunctionState::DisjunctionState(unsigned generation) : _generation(generation) { }
void DisjunctionState::accumulate(PredicateDomain::element_type *head, LitVec const &lits) {
    ULitVec uLits;
    for (Literal &x : lits) { uLits.emplace_back(x.clone()); }
    elems.emplace_back(head, std::move(uLits));
}
bool DisjunctionState::defined() const { return true; }
unsigned DisjunctionState::generation() const { return _generation; }
bool DisjunctionState::fact(bool) const { return false; }
DisjunctionState::element_type &DisjunctionState::ignore() { throw std::logic_error("DisjunctionState::ignore must not be called"); }

// }}}
// {{{ definition of DisjunctionRule

void DisjunctionRule::toLparse(LparseTranslator &x) {
    SAuxAtom            bdAtom;
    SAuxAtomVec         djAuxHead;
    LparseRule::HeadVec djHead;
    if (body.empty()) {
        Value value;
        auto isBound = [&]() -> bool {
            if (repr->elems.empty()) { return false; }
            for (auto &y : repr->elems) {
                if (y.first != nullptr) { return false; }
                if (y.second.empty())   { return false; }
                for (auto &z : y.second) {
                    if (!z->isBound(value, false)) { return false; }
                }
            }
            return true;
        };
        if (isBound()) {
            std::vector<CSPBound> bounds;
            for (auto &y : repr->elems) {
                bounds.emplace_back(std::numeric_limits<int>::min(), std::numeric_limits<int>::max()-1);
                for (auto &z : y.second) { z->updateBound(bounds.back(), false); }
            }
            x.addBounds(value, bounds);
            return;
        }
    }
    for (auto &y : repr->elems) {
        if (y.first && y.second.empty()) { djHead.emplace_back(*y.first); }
        else {
            // cdAtom :- y.second.
            ULit cdLit;
            ULit ncdLit;
            if (y.second.size() == 1) { 
                cdLit = std::move(y.second.front()); 
                SAuxAtom ncdAtom(std::make_shared<AuxAtom>(x.auxAtom()));
                LparseRule(ncdAtom, get_clone(cdLit), nullptr).toLparse(x);
                ncdLit = make_unique<AuxLiteral>(ncdAtom, true);
            }
            else {
                SAuxAtom cdAtom(std::make_shared<AuxAtom>(x.auxAtom()));
                LparseRule(cdAtom, std::move(y.second)).toLparse(x);
                cdLit  = make_unique<AuxLiteral>(cdAtom, false);
                ncdLit = make_unique<AuxLiteral>(cdAtom, true);
            }
            if (y.first) {
                SAuxAtom hdAtom(std::make_shared<AuxAtom>(x.auxAtom()));
                // y.first :- hdAtom, cdLit.
                LparseRule(*y.first, make_unique<AuxLiteral>(hdAtom, false), get_clone(cdLit)).toLparse(x);
                // hdAtom  :- y.first, cdLit.
                LparseRule(hdAtom, make_unique<PredicateLiteral>(NAF::POS, *y.first), std::move(cdLit)).toLparse(x);
                // :- hdAtom, not cdLit.
                LparseRule(make_unique<AuxLiteral>(hdAtom, false), std::move(ncdLit)).toLparse(x);
                djAuxHead.emplace_back(hdAtom);
            }
            else { body.emplace_back(std::move(ncdLit)); }
        }
    }
    LparseRule(std::move(djHead), std::move(djAuxHead), std::move(body), false).toLparse(x);
}
void DisjunctionRule::printLparse(LparseOutputter &) const {
    throw std::logic_error("DisjunctionRule::toLparse must be called before DisjunctionRule::printLparse");
}
void DisjunctionRule::printElem(std::ostream &out, DisjunctionState::ElemSet::value_type const &x) {
    if (x.first) { out << x.first->first; }
    else         { out << "#true"; }
    if (!x.second.empty()) {
        out << ":";
        using namespace std::placeholders;
        print_comma(out, x.second, ",", std::bind(&Literal::printPlain, _2, _1));
    }
}
void DisjunctionRule::printPlain(std::ostream &out) const {
    print_comma(out, repr->elems, ";", &DisjunctionRule::printElem);
    if (!body.empty()) {
        out << ":-";
        using namespace std::placeholders;
        print_comma(out, body, ";", std::bind(&Literal::printPlain, _2, _1));
    }
    out << ".\n";
}
bool DisjunctionRule::isIncomplete() const { return true; }
DisjunctionRule *DisjunctionRule::clone() const { 
    auto ret(make_unique<DisjunctionRule>());
    ret->repr = repr;
    ret->body = get_clone(body);
    return ret.release();
}
DisjunctionRule::~DisjunctionRule() { }

// }}}
// {{{ definition of Minimize

void Minimize::toLparse(LparseTranslator &x) {
    for (auto &y : elems) {
        for (auto &z : y.second) { Term::replace(z, z->toLparse(x)); }
    }
    x.addMinimize(std::move(elems));
}
void Minimize::printPlain(std::ostream &out) const {
    for (auto &x : elems) {
        out << ":~";
        using namespace std::placeholders;
        print_comma(out, x.second, ";", std::bind(&Literal::printPlain, _2, _1));
        out << ".[";
        auto it(x.first.begin());
        out << *it++ << "@";
        out << *it++;
        for (auto ie(x.first.end()); it != ie; ++it) { out << "," << *it; }
        out << "]\n";
    }
}
void Minimize::printLparse(LparseOutputter &) const {
    throw std::runtime_error("Minimize::printLparse: must not be called");
}
bool Minimize::isIncomplete() const { return false; }
Minimize *Minimize::clone() const { throw std::logic_error("Minimize::clone must not be called."); }
Minimize::~Minimize() { }

// }}}
// {{{ definition of LparseMinimize

LparseMinimize::LparseMinimize(Value prio, ULitWeightVec &&lits)
    : prio(prio)
    , lits(std::move(lits)) { }
void LparseMinimize::toLparse(LparseTranslator &x) {
    for (auto &y : lits) { Term::replace(y.first, y.first->toLparse(x)); }
    x(*this);
}
void LparseMinimize::printPlain(std::ostream &out) const {
    int i = 0;
    out << "#minimize{";
    auto f = [&i, this](std::ostream &out, ULitWeightVec::value_type const &x) { out << x.second << "@" << prio << "," << i++ << ":"; x.first->printPlain(out); };
    print_comma(out, lits, ";", f);
    out << "}.\n";
}
void LparseMinimize::printLparse(LparseOutputter &x) const {
    LparseOutputter::LitWeightVec body;
    for (auto &y : lits) { body.emplace_back(y.first->lparseUid(x), y.second); }
    x.printMinimize(body);
}
bool LparseMinimize::isIncomplete() const { return false; }
LparseMinimize *LparseMinimize::clone() const { throw std::logic_error("LparseMinimize::clone must not be called."); }
LparseMinimize::~LparseMinimize() { }

// }}}

} } // namespace Output Gringo
