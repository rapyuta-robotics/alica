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
#include <gringo/logger.hh>

namespace Gringo { namespace Output {

// {{{ definition of AuxLiteral

AuxAtom::AuxAtom(unsigned name) : name(name) { }
int AuxAtom::lparseUid(LparseOutputter &out) {
    if (!uid) { uid = out.newUid(); }
    return uid;
}

std::ostream &operator<<(std::ostream &out, AuxAtom const &x) { 
    out << "#aux(" << x.name << ")";
    return out;
}

AuxLiteral::AuxLiteral(SAuxAtom atom, bool negative) : atom(atom), negative(negative) { }
AuxLiteral *AuxLiteral::clone() const                             { return new AuxLiteral(*this); }
ULit AuxLiteral::toLparse(LparseTranslator &)                     { return nullptr; }
void AuxLiteral::makeEqual(ULit &&lit, LparseTranslator &x) const {
    if (!negative) { LparseRule(atom, std::move(lit), nullptr).toLparse(x); }
}
void AuxLiteral::printPlain(std::ostream &out) const              { out << (negative ? "not " : "") << *atom; }
bool AuxLiteral::isIncomplete() const                             { return false; }
int AuxLiteral::lparseUid(LparseOutputter &out) const       { return !negative ? atom->lparseUid(out) : -atom->lparseUid(out); }
size_t AuxLiteral::hash() const                                   { return get_value_hash(typeid(AuxLiteral).hash_code(), atom->name); }
bool AuxLiteral::operator==(Literal const &x) const               { 
    AuxLiteral const *t{dynamic_cast<AuxLiteral const*>(&x)};
    return negative == t->negative && atom->name == t->atom->name;
}
AuxLiteral::~AuxLiteral()                                         { }

// }}}
// {{{ definition of BooleanLiteral

BooleanLiteral::BooleanLiteral(bool value) : value(value)         { }
BooleanLiteral *BooleanLiteral::clone() const                     { return new BooleanLiteral(*this); }
BooleanLiteral::~BooleanLiteral()                                 { }
ULit BooleanLiteral::toLparse(LparseTranslator &)                 { return nullptr; }
void BooleanLiteral::makeEqual(ULit &&, LparseTranslator &) const { }
void BooleanLiteral::printPlain(std::ostream &out) const          { out << (value ? "#true" : "#false"); }
bool BooleanLiteral::isIncomplete() const                         { return false; }
int BooleanLiteral::lparseUid(LparseOutputter &out) const         { return value ? -out.falseUid() : out.falseUid(); }
size_t BooleanLiteral::hash() const                               { return get_value_hash(typeid(BooleanLiteral).hash_code(), value); }
bool BooleanLiteral::operator==(Literal const &x) const           { 
    BooleanLiteral const *t{dynamic_cast<BooleanLiteral const*>(&x)};
    return value == t->value;
}

// }}}
// {{{ definition of PredicateLiteral

PredicateLiteral::PredicateLiteral() = default;
PredicateLiteral::PredicateLiteral(NAF naf, PredicateDomain::element_type &repr) : naf(naf), repr(&repr) { }
void PredicateLiteral::printPlain(std::ostream &out) const {
    out << naf << repr->first;
}
bool PredicateLiteral::isIncomplete() const         { return false; }
PredicateLiteral *PredicateLiteral::clone() const   { return new PredicateLiteral(*this); }
ULit PredicateLiteral::toLparse(LparseTranslator &) { return nullptr; }
void PredicateLiteral::makeEqual(ULit &&lit, LparseTranslator &x) const {
    if (naf == NAF::POS) { 
        ULitVec lits;
        lits.emplace_back(std::move(lit));
        Rule(repr, std::move(lits)).toLparse(x);
    }
}
int PredicateLiteral::lparseUid(LparseOutputter &out) const {
    if (!repr->second.hasUid()) { repr->second.uid(out.newUid()); }
    switch (naf) {
        case NAF::POS: { return +repr->second.uid(); }
        case NAF::NOT: { return -int(repr->second.uid()); }
        case NAF::NOTNOT: { 
            int aux(out.newUid());
            LparseOutputter::LitVec lits;
            lits.emplace_back(-int(repr->second.uid()));
            out.printBasicRule(aux, lits);
            return -aux;
        }
    }
    assert(false);
    return 0;
}
bool PredicateLiteral::invertible() const {
    return naf != NAF::POS;
}
void PredicateLiteral::invert() {
    naf = inv(naf);
}
size_t PredicateLiteral::hash() const { return get_value_hash(typeid(PredicateLiteral).hash_code(), naf, repr->first); }
bool PredicateLiteral::operator==(Literal const &x) const { 
    PredicateLiteral const *t{dynamic_cast<PredicateLiteral const*>(&x)};
    return naf == t->naf && repr == t->repr;
}
PredicateLiteral::~PredicateLiteral() { }

// }}}
// {{{ definition of BodyAggregateState

int clamp(int64_t x) {
    if (x > std::numeric_limits<int>::max()) { return std::numeric_limits<int>::max(); }
    if (x < std::numeric_limits<int>::min()) { return std::numeric_limits<int>::min(); }
    return int(x);
}

bool neutral(ValVec const &tuple, AggregateFunction fun, Location const &loc) {
    if (tuple.empty()) { 
        if (fun == AggregateFunction::COUNT) { return false; }
        else {
            GRINGO_REPORT(W_TERM_UNDEFINED) 
                << loc << ": warning: empty tuple in " << fun << " aggregate, tuple is ignored\n";
            return true;
        }
    }
    else if (tuple.front().type() != Value::SPECIAL) {
        bool ret = true;
        switch (fun) {
            case AggregateFunction::MIN:   { return tuple.front() == Value(false); }
            case AggregateFunction::MAX:   { return tuple.front() == Value(true); }
            case AggregateFunction::COUNT: { return false; }
            case AggregateFunction::SUM:   { ret = tuple.front().type() != Value::NUM || tuple.front() == 0; break; }
            case AggregateFunction::SUMP:  { ret = tuple.front().type() != Value::NUM || tuple.front() <= 0; break; }
        }
        if (ret && tuple.front() != 0) {
            GRINGO_REPORT(W_TERM_UNDEFINED) 
                << loc << ": warning: " << fun <<  " aggregate not defined for weight, tuple is ignored:\n"
                << "  " << tuple.front() << "\n";
        }
        return ret;
    }
    return true;
}

int toInt(IntervalSet<Value>::LBound const &x) {
    if (x.bound.type() == Value::NUM) {
        return x.inclusive ? x.bound.num() : x.bound.num() + 1;
    }
    else {
        if (x.bound < 0) { return std::numeric_limits<int>::min(); }
        else             { return std::numeric_limits<int>::max(); }
    }
}

int toInt(IntervalSet<Value>::RBound const &x) {
    if (x.bound.type() == Value::NUM) {
        return x.inclusive ? x.bound.num() : x.bound.num() - 1;
    }
    else {
        if (x.bound < 0) { return std::numeric_limits<int>::min(); }
        else             { return std::numeric_limits<int>::max(); }
    }
}

Value getWeight(AggregateFunction fun, FWValVec const &x) {
    return fun == AggregateFunction::COUNT ? Value(1) : *x.begin();
}

bool BodyAggregateState::fact(bool recursive) const { return _fact && (_positive || !recursive); }
unsigned BodyAggregateState::generation() const { return _generation; }
bool BodyAggregateState::isFalse() { return state != DEFINED; }
BodyAggregateState::element_type &BodyAggregateState::ignore() {
    static element_type x{std::piecewise_construct, std::forward_as_tuple(Value("#false")), std::forward_as_tuple()};
    return x;
}
void BodyAggregateState::accumulate(ValVec const &tuple, AggregateFunction fun, bool fact, bool remove) {
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
void BodyAggregateState::init(AggregateFunction fun) {
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
bool BodyAggregateState::defined() const { return state == DEFINED; }
void BodyAggregateState::generation(unsigned x) { _generation = x; }
BodyAggregateState::Bounds::Interval BodyAggregateState::range(AggregateFunction fun) const { 
    if (fun != AggregateFunction::MIN && fun != AggregateFunction::MAX) {
        return {{clamp(intMin), true}, {clamp(intMax), true}};
    }
    else { return {{valMin, true}, {valMax, true}}; }
}
BodyAggregateState::~BodyAggregateState() { }

// }}}
// {{{ definition of BodyAggregate

BodyAggregate::BodyAggregate(Location const *&loc) : loc(loc) { }

namespace {

void print_elem(std::ostream &out, BdAggrElemSet::value_type const &x) {
    if (x.second.empty()) { print_comma(out, x.first, ","); }
    else {
        auto print_cond = [&x](std::ostream &out, BdAggrElemSet::value_type::second_type::value_type const &y) -> void { 
            print_comma(out, x.first, ",");
            out << ":";
            using namespace std::placeholders;
            print_comma(out, y, ",", std::bind(&Literal::printPlain, _2, _1));
        };
        print_comma(out, x.second, ";", print_cond);
    }
}

} // namespace

void BodyAggregate::printPlain(std::ostream &out) const {
    out << naf;
    auto it = bounds.begin(), ie = bounds.end();
    if (it != ie) { out << it->second << inv(it->first); ++it; }
    out << fun << "{";
    print_comma(out, repr->second.elems, ";", print_elem);
    out << "}";
    for (; it != ie; ++it) { out << it->first << it->second; }

}
ULit BodyAggregate::toLparse(LparseTranslator &x) {
    // TODO: this algorithm suffers from severe uglyness!!!
    // TODO: check if false
    auto rng(repr->second.range(fun));
    SAuxAtom auxAtom(std::make_shared<AuxAtom>(x.auxAtom()));
    if (repr->second.bounds.contains(rng)) { LparseRule(auxAtom, ULitVec()).toLparse(x); } 
    else {
        using ULitValVec = std::vector<std::pair<ULit,Value>>;
        ULitValVec elemVec;
        for (auto &y : repr->second.elems) { 
            Value weight(getWeight(fun, y.first));
            if (y.second.size() != 1 || y.second.front().size() != 1) {
                SAuxAtom atom(std::make_shared<AuxAtom>(x.auxAtom()));
                elemVec.emplace_back(make_unique<AuxLiteral>(atom, false), weight);
                // atom :- cond
                if (!y.second.empty()) {
                    std::vector<ULitVec> &conds(y.second);
                    for (ULitVec &y : conds) {
                        LparseRule(atom, get_clone(y)).toLparse(x);
                    }
                }
                else {
                    LparseRule(atom, {}).toLparse(x);
                }
            }
            else { elemVec.emplace_back(get_clone(y.second.front().front()), weight); }
        }
        if (incomplete && loc && repr->second.bounds.vec.size() > 1) {
            GRINGO_REPORT(W_NONMONOTONE_AGGREGATE) 
                << *loc << ": warning: holes in range of (potentially) recursive " << fun << " aggregate:\n"
                << "  (the applied translation might produce counter-intuitive results)\n";
            loc = nullptr;
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
                        if (incomplete && loc) {
                            GRINGO_REPORT(W_NONMONOTONE_AGGREGATE) 
                                << *loc << ": warning: negative weight in (potentially) recursive " << fun << " aggregate:\n"
                                << "  (the applied translation might produce counter-intuitive results)\n";
                            loc = nullptr;
                        }
                    }
                    else { elemPos.emplace_back(get_clone(y.first), weight); }
                }
                for (auto &y : repr->second.bounds.vec) {
                    ULitVec aggrLits;
                    if (rng.left < y.left) {
                        // aggr :- L { }.
                        SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                        WeightRule wr(aggr, std::max(0, toInt(y.left) + shift), get_clone(elemPos));
                        wr.toLparse(x);
                        aggrLits.emplace_back(make_unique<AuxLiteral>(aggr, naf == NAF::NOTNOT));
                    }
                    if (y.right < rng.right) {
                        // aggr :- U+1 { }.
                        SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                        WeightRule wr(aggr, std::max(0, toInt(y.right) + 1 + shift), get_clone(elemPos));
                        wr.toLparse(x);
                        aggrLits.emplace_back(make_unique<AuxLiteral>(aggr, naf != NAF::NOTNOT));
                    }
                    LparseRule(auxAtom, std::move(aggrLits)).toLparse(x);
                }
                break;
            }
            case AggregateFunction::MIN:
            case AggregateFunction::MAX: {
                for (auto &y : repr->second.bounds) {
                    ULitVec aggrLits;
                    if (fun == AggregateFunction::MIN ? y.right < rng.right : rng.left < y.left) {
                        SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                        for (auto &z : elemVec) {
                            // aggr :- 1 { between }
                            if (y.contains(z.second)) { LparseRule(aggr, get_clone(z.first), nullptr).toLparse(x); }
                        }
                        aggrLits.emplace_back(make_unique<AuxLiteral>(aggr, naf == NAF::NOTNOT));
                    }
                    if (fun == AggregateFunction::MIN ? rng.left < y.left : y.right < rng.right) {
                        SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                        for (auto &z : elemVec) {
                            if (fun == AggregateFunction::MIN ? z.second < y : y < z.second) {
                                // if min:  aggr :- 1 { below }
                                // eif max: aggr :- 1 { above }
                                LparseRule(aggr, get_clone(z.first), nullptr).toLparse(x);
                            }
                        }
                        aggrLits.emplace_back(make_unique<AuxLiteral>(aggr, naf != NAF::NOTNOT));
                    }
                    LparseRule(auxAtom, std::move(aggrLits)).toLparse(x);
                }
                break;
            }
        }
    }
    return make_unique<AuxLiteral>(auxAtom, naf != NAF::POS);
}
void BodyAggregate::makeEqual(ULit &&, LparseTranslator &) const {
    throw std::runtime_error("BodyAggregate::makeEqual: must not happen!!!");
}
int BodyAggregate::lparseUid(LparseOutputter &) const {
    throw std::runtime_error("BodyAggregate::lparseUid must be called after BodyAggregate::toLparse");
}
bool BodyAggregate::isIncomplete() const { return incomplete; }
BodyAggregate *BodyAggregate::clone() const { return new BodyAggregate(*this); }
size_t BodyAggregate::hash() const { throw std::runtime_error("BodyAggregate::hash: implement me if necessary!"); }
bool BodyAggregate::operator==(Literal const &) const { throw std::runtime_error("BodyAggregate::operator==: implement me if necessary!"); }
BodyAggregate::~BodyAggregate() { }

// }}}
// {{{ definition of AssignmentAggregateState

AssignmentAggregateState::AssignmentAggregateState(Data *data, unsigned generation)
    : data(data)
    , _generation(generation)   { }
bool AssignmentAggregateState::fact(bool recursive) const { return data->fact && !recursive; }
unsigned AssignmentAggregateState::generation() const     { return _generation; }
void AssignmentAggregateState::generation(unsigned x)     { _generation = x; }
bool AssignmentAggregateState::isFalse()                  { return data; }
AssignmentAggregateState::element_type &AssignmentAggregateState::ignore()   { throw std::logic_error("AssignmentAggregateState::ignore must not be called"); }
bool AssignmentAggregateState::defined() const            { return true; }

// }}}
// {{{ definition of AssignmentAggregate

AssignmentAggregate::AssignmentAggregate(Location const *&loc) : loc(loc) { }
void AssignmentAggregate::printPlain(std::ostream &out) const {
    out << *(repr->first.args().end()-1) << "=" << fun << "{";
    print_comma(out, repr->second.data->elems, ";", print_elem);
    out << "}";
}
ULit AssignmentAggregate::toLparse(LparseTranslator &x) {
    // TODO: this algorithm suffers from severe uglyness!!!
    // TODO: check if false
    SAuxAtom auxAtom(std::make_shared<AuxAtom>(x.auxAtom()));
    using ULitValVec = std::vector<std::pair<ULit,Value>>;
    ULitValVec elemVec;
    for (auto &y : repr->second.data->elems) { 
        Value weight(getWeight(fun, y.first));
        if (y.second.size() != 1 || y.second.front().size() != 1) {
            SAuxAtom atom(std::make_shared<AuxAtom>(x.auxAtom()));
            elemVec.emplace_back(make_unique<AuxLiteral>(atom, false), weight);
            // atom :- cond
            if (!y.second.empty()) {
                std::vector<ULitVec> &conds(y.second);
                for (auto &y : conds) {
                    LparseRule br(atom, get_clone(y));
                    br.toLparse(x);
                }
            }
            else {
                LparseRule br(atom, {});
                br.toLparse(x);
            }
        }
        else { elemVec.emplace_back(get_clone(y.second.front().front()), weight); }
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
                    ULitVec lits;
                    lits.emplace_back(get_clone(y.first));
                    LparseRule br(neg, std::move(lits));
                    br.toLparse(x);
                    elemPos.emplace_back(make_unique<AuxLiteral>(neg, true), -weight);
                    if (incomplete && loc) {
                        GRINGO_REPORT(W_NONMONOTONE_AGGREGATE) 
                            << *loc << ": warning: negative weight in (potentially) recursive " << fun << " aggregate:\n"
                            << "  (the applied translation might produce counter-intuitive results)\n";
                        loc = nullptr;
                    }
                }
                else { elemPos.emplace_back(get_clone(y.first), weight); }
            }
            int assign((*(repr->first.args().end()-1)).num());
            ULitVec aggrLits;
            {
                // aggr :- L { }.
                SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                WeightRule wr(aggr, std::max(0, assign + shift), get_clone(elemPos));
                wr.toLparse(x);
                aggrLits.emplace_back(make_unique<AuxLiteral>(aggr, false));
            }
            {
                // aggr :- U+1 { }.
                SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                WeightRule wr(aggr, std::max(0, assign + 1 + shift), get_clone(elemPos));
                wr.toLparse(x);
                aggrLits.emplace_back(make_unique<AuxLiteral>(aggr, true));
            } 
            LparseRule br(auxAtom, std::move(aggrLits));
            br.toLparse(x);
            break;
        }
        case AggregateFunction::MIN:
        case AggregateFunction::MAX: {
            Value assign(*(repr->first.args().end()-1));
            ULitVec aggrLits;
            {
                // aggr :- 1 { between }
                if (assign != (fun == AggregateFunction::MIN ? Value(false) : Value(true))) {
                    SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                    for (auto &z : elemVec) {
                        if (z.second == assign) {
                            ULitVec lits;
                            lits.emplace_back(get_clone(z.first));
                            LparseRule br(aggr, std::move(lits));
                            br.toLparse(x);
                        }
                    }
                    aggrLits.emplace_back(make_unique<AuxLiteral>(aggr, false));
                }
            }
            {
                SAuxAtom aggr(std::make_shared<AuxAtom>(x.auxAtom()));
                for (auto &z : elemVec) {
                    if (fun == AggregateFunction::MIN ? z.second < assign : assign < z.second) {
                        // if min:  aggr :- 1 { below }
                        // eif max: aggr :- 1 { above }
                        ULitVec lits;
                        lits.emplace_back(get_clone(z.first));
                        LparseRule br(aggr, std::move(lits));
                        br.toLparse(x);
                    }
                }
                aggrLits.emplace_back(make_unique<AuxLiteral>(aggr, true));
            } 
            LparseRule br(auxAtom, std::move(aggrLits));
            br.toLparse(x);
            break;
        }
    }
    return make_unique<AuxLiteral>(auxAtom, false);
}
void AssignmentAggregate::makeEqual(ULit &&, LparseTranslator &) const { throw std::runtime_error("AssignmentAggregate::makeEqual: must not happen!!!"); }
int AssignmentAggregate::lparseUid(LparseOutputter &) const            { throw std::runtime_error("AssignmentAggregate::lparseUid must be called after AssignmentAggregate::toLparse"); }
bool AssignmentAggregate::isIncomplete() const                         { return incomplete; }
AssignmentAggregate *AssignmentAggregate::clone() const                { return new AssignmentAggregate(*this); }
size_t AssignmentAggregate::hash() const                               { throw std::runtime_error("AssignmentAggregate::hash: implement me if necessary!"); }
bool AssignmentAggregate::operator==(Literal const &) const            { throw std::runtime_error("AssignmentAggregate::operator==: implement me if necessary!"); }
AssignmentAggregate::~AssignmentAggregate()                            { }

// }}}
// {{{ definition of ConjunctionElem

ConjunctionElem::ConjunctionElem(PredicateDomain::element_type* head, ULitVec &&body)
    : head(head)
    , body(std::move(body)) { }

size_t ConjunctionElem::hash() const {
    return get_value_hash(size_t(head), body);
}

bool ConjunctionElem::operator==(ConjunctionElem const &x) const {
    return head == x.head && is_value_equal_to(body, x.body);
}

void ConjunctionElem::print(std::ostream &out) const {
    if (head) { out << head->first; }
    else      { out << "#false"; }
    if (!body.empty()) {
        out << ":";
        using namespace std::placeholders;
        print_comma(out, body, ",", std::bind(&Literal::printPlain, _2, _1));
    }
}

// }}}
// {{{ definition of ConjunctionState

bool ConjunctionState::fact(bool recursive) const          { return _fact && !recursive; } // TODO: likely not what I want!!!
unsigned ConjunctionState::generation() const              { return _generation - 2; }
void ConjunctionState::generation(unsigned x)              { _generation = x + 2; }
bool ConjunctionState::isFalse()                           { throw std::logic_error("ConjunctionState::isFalse must not be called"); }
ConjunctionState::element_type &ConjunctionState::ignore() { throw std::logic_error("ConjunctionState::ignore must not be called"); }
bool ConjunctionState::defined() const                     { return _generation > 1; }

// }}}
// {{{ definition of Conjunction

void Conjunction::printPlain(std::ostream &out) const {
    if (!repr->second.elems.empty()) {
        print_comma(out, repr->second.elems, ";");
    }
    else { out << "#true"; }
}
ULit Conjunction::toLparse(LparseTranslator &x) {
    // TODO: at some point I should try to prove this translation
    if (!repr->second.bdLit) {
        repr->second.bdLit = std::make_shared<AuxAtom>(x.auxAtom());
        ULitVec bd;
        for (ConjunctionState::Elem &y : repr->second.elems) {
            if (!y.head && y.body.size() == 1 && y.body.front()->invertible()) {
                ULit inv(get_clone(y.body.front()));
                inv->invert();
                bd.emplace_back(std::move(inv));
            }
            else if (!y.body.empty()) {
                SAuxAtom aux(std::make_shared<AuxAtom>(x.auxAtom()));
                if (y.head) {
                    // aux :- x.first.
                    ULitVec lits;
                    lits.emplace_back(make_unique<PredicateLiteral>(NAF::POS, *y.head));
                    LparseRule(aux, std::move(lits)).toLparse(x);
                }
                SAuxAtom chk(std::make_shared<AuxAtom>(x.auxAtom()));
                // chk :- y.second.
                ULitVec lits;
                for (auto &z : y.body) { lits.emplace_back(get_clone(z)); }
                LparseRule(chk, std::move(lits)).toLparse(x);
                // aux :- ~chk.
                lits.emplace_back(make_unique<AuxLiteral>(chk, true));
                LparseRule(aux, std::move(lits)).toLparse(x);
                if (incomplete && y.head) {
                    // aux | chk :- ~~x.first.
                    SAuxAtomVec head;
                    head.emplace_back(aux);
                    head.emplace_back(chk);
                    lits.emplace_back(make_unique<PredicateLiteral>(NAF::NOTNOT, *y.head));
                    LparseRule(std::move(head), std::move(lits), false).toLparse(x);
                    // y.second :- chk.
                    for (auto &z : y.body) { z->makeEqual(make_unique<AuxLiteral>(chk, false), x); }
                }
                // body += aux
                bd.emplace_back(make_unique<AuxLiteral>(aux, false));
            }
            // body += x.first
            else if (y.head) { bd.emplace_back(make_unique<PredicateLiteral>(NAF::POS, *y.head)); }
            else             { return make_unique<AuxLiteral>(repr->second.bdLit, false); }
        }
        // bdLit :- body.
        LparseRule(repr->second.bdLit, std::move(bd)).toLparse(x);
    }
    return make_unique<AuxLiteral>(repr->second.bdLit, false);
}
void Conjunction::makeEqual(ULit &&, LparseTranslator &) const {
    throw std::runtime_error("AssignmentAggregate::makeEqual: must not happen!!!");
}
int Conjunction::lparseUid(LparseOutputter &) const { throw std::logic_error("Conjunction::toLparse: must be called before Conjunction::lparseUid"); }
bool Conjunction::isIncomplete() const                    { return incomplete; }
Conjunction *Conjunction::clone() const                   { return new Conjunction(*this); }
size_t Conjunction::hash() const                          { throw std::runtime_error("Conjunction::hash: implement me if necessary!"); }
bool Conjunction::operator==(Literal const &) const       { throw std::runtime_error("Conjunction::operator==: implement me if necessary!"); }
Conjunction::~Conjunction()                               { }

// }}}
// {{{ definition of DisjointState

DisjointElem::DisjointElem(CSPGroundAdd &&value, int fixed, ULitVec &&lits)
    : value(std::move(value))
    , fixed(fixed)
    , lits(std::move(lits)) { }
DisjointElem::DisjointElem(DisjointElem &&) = default;
DisjointElem DisjointElem::clone() const { return DisjointElem(get_clone(value), fixed, get_clone(lits)); }
DisjointElem::~DisjointElem() { }

bool DisjointState::fact(bool recursive) const { return !recursive && elems.size() <= 1; }
unsigned DisjointState::generation() const     { return _generation - 1; }
bool DisjointState::isFalse()                  { return false; }
DisjointState::element_type &DisjointState::ignore() {
    static element_type x{std::piecewise_construct, std::forward_as_tuple(Value("#false")), std::forward_as_tuple()};
    return x;
}
bool DisjointState::defined() const            { return _generation > 0; }
void DisjointState::generation(unsigned x)     { _generation = x + 1; }
DisjointState::~DisjointState()                { }

// }}}
// {{{ definition of DisjointLiteral

DisjointLiteral::DisjointLiteral(NAF naf) : naf(naf) { }

void DisjointLiteral::printPlain(std::ostream &out) const {
    auto print_elem = [](std::ostream &out, DisjointElemSet::value_type const &x) {
        assert(!x.second.empty());
        auto print_cond = [&x](std::ostream &out, DisjointElem const &y) {
            print_comma(out, x.first, ",");
            out << ":";
            auto print_add = [](std::ostream &out, CSPGroundAdd::value_type const &z) {
                if (z.first == 1) { out              << "$" << z.second; }
                else              { out << z.first << "$*$" << z.second; }
            };
            if (y.value.empty()) { out << y.fixed; }
            else {
                print_comma(out, y.value, "$+", print_add);
                if (y.fixed > 0)      { out << "$+" << y.fixed; }
                else if (y.fixed < 0) { out << "$-" << -y.fixed; }
            }
            if (!y.lits.empty()) {
                out << ":";
                using namespace std::placeholders;
                print_comma(out, y.lits, ",", std::bind(&Literal::printPlain, _2, _1));
            }
        };
        print_comma(out, x.second, ";", print_cond);
    };
    out << naf;
    out << "#disjoint{";
    print_comma(out, repr->second.elems, ";", print_elem);
    out << "}";
}
ULit DisjointLiteral::toLparse(LparseTranslator &x) {
    SAuxAtom aux(std::make_shared<AuxAtom>(x.auxAtom()));
    DisjointCons cons;
    for (auto &y : repr->second.elems) { 
        cons.emplace_back(std::piecewise_construct, std::forward_as_tuple(y.first), std::forward_as_tuple());
        for (auto &z : y.second) {
            ULitVec cond;
            for (auto &lit : z.lits) {
                cond.emplace_back(lit->toLparse(x));
                if (!cond.back()) { cond.back() = std::move(lit); }
            }
            cons.back().second.emplace_back(std::move(z.value), z.fixed, std::move(cond));
        }
    }
    x.addDisjointConstraint(aux, std::move(cons));
    if (naf == NAF::NOT) {
        return make_unique<AuxLiteral>(aux, true);
    }
    else {
        SAuxAtom neg(std::make_shared<AuxAtom>(x.auxAtom()));
        LparseRule(neg, make_unique<AuxLiteral>(aux, true), nullptr).toLparse(x);
        return make_unique<AuxLiteral>(neg, true);
    }
}
void DisjointLiteral::makeEqual(ULit &&, LparseTranslator &) const {
    throw std::runtime_error("DisjointLiteral::makeEqual: must not happen!!!");
}
int DisjointLiteral::lparseUid(LparseOutputter &) const {
    throw std::runtime_error("DisjointLiteral::lparseUid must be called after DisjointLiteral::toLparse");
}
bool DisjointLiteral::isIncomplete() const { return incomplete; }
DisjointLiteral *DisjointLiteral::clone() const { return new DisjointLiteral(*this); }
size_t DisjointLiteral::hash() const { throw std::runtime_error("DisjointLiteral::hash: implement me if necessary!"); }
bool DisjointLiteral::operator==(Literal const &) const { throw std::runtime_error("DisjointLiteral::operator==: implement me if necessary!"); }
DisjointLiteral::~DisjointLiteral() { }

// }}}
// {{{ definition of CSPLiteral

CSPLiteral::CSPLiteral() = default;

void CSPLiteral::reset(CSPGroundLit &&ground) {
    this->ground = std::move(ground);
}
void CSPLiteral::printPlain(std::ostream &out) const {
    if (!std::get<1>(ground).empty()) {
        auto f = [](std::ostream &out, std::pair<int, Value> mul) { out << mul.first << "$*$" << mul.second; };
        print_comma(out, std::get<1>(ground), "$+", f);
    }
    else { out << 0; }
    out << "$" << std::get<0>(ground);
    out << std::get<2>(ground);
}
bool CSPLiteral::isIncomplete() const {
    return false;
}
CSPLiteral *CSPLiteral::clone() const {
    return new CSPLiteral(*this);
}
size_t CSPLiteral::hash() const {
    return get_value_hash(typeid(CSPLiteral).hash_code(), ground);
}
bool CSPLiteral::operator==(Literal const &x) const {
    CSPLiteral const *t = dynamic_cast<CSPLiteral const*>(&x);
    return t && is_value_equal_to(ground, t->ground);
}
bool CSPLiteral::isBound(Value &value, bool negate) const {
    Relation rel = std::get<0>(ground);
    if (negate) { rel = neg(rel); }
    if (std::get<1>(ground).size() != 1) { return false; }
    if (rel == Relation::NEQ)            { return false; }
    if (value.type() == Value::SPECIAL)  { value = std::get<1>(ground).front().second; }
    return value == std::get<1>(ground).front().second;
}
void CSPLiteral::updateBound(CSPBound &bound, bool negate) const {
    Relation rel = std::get<0>(ground);
    if (negate) { rel = neg(rel); }
    int coef  = std::get<1>(ground).front().first;
    int fixed = std::get<2>(ground);
    if (coef < 0) { 
        coef  = -coef;
        fixed = -fixed;
        rel   = inv(rel);
    }
    switch (rel) {
        case Relation::LT:  { fixed--; }
        case Relation::LEQ: {
            fixed /= coef;
            bound.second = std::min(bound.second, fixed);
            break;
        }
        case Relation::GT:  { fixed++; }
        case Relation::GEQ: {
            fixed = (fixed+coef-1) / coef;
            bound.first  = std::max(bound.first,  fixed);
            break;
        }
        case Relation::EQ:
        case Relation::ASSIGN: {
            if (fixed % coef == 0) {
                fixed /= coef;
                bound.first  = std::max(bound.first,  fixed);
                bound.second = std::min(bound.second, fixed);
            }
            else {
                bound.first  = 0;
                bound.second = -1;
            }
            break;
        }
        case Relation::NEQ: { assert(false); }
    }
}
ULit CSPLiteral::toLparse(LparseTranslator &x) {
    Relation rel = std::get<0>(ground);
    int bound    = std::get<2>(ground);
    auto addInv = [&x](SAuxAtom aux, CoefVarVec &&vars, int bound) {
        for (auto &x : vars) { x.first *= -1; }
        x.addLinearConstraint(aux, std::move(vars), -bound);
    };
    SAuxAtom aux(std::make_shared<AuxAtom>(x.auxAtom()));
    switch (rel) {
        case Relation::EQ:
        case Relation::ASSIGN:
        case Relation::NEQ: {
            x.addLinearConstraint(aux, CoefVarVec(std::get<1>(ground)), bound-1);
            addInv(aux, std::move(std::get<1>(ground)), bound + 1);
            return make_unique<AuxLiteral>(aux, rel != Relation::NEQ);
        }
        case Relation::LT:  { bound--; }
        case Relation::LEQ: {
            x.addLinearConstraint(aux, CoefVarVec(std::get<1>(ground)), bound);
            return make_unique<AuxLiteral>(aux, false);
        }
        case Relation::GT:  { bound++; }
        case Relation::GEQ: {
            addInv(aux, std::move(std::get<1>(ground)), bound);
            return make_unique<AuxLiteral>(aux, false);
        }
    }
    assert(false);
    return nullptr;
}
void CSPLiteral::makeEqual(ULit &&, LparseTranslator &) const {
    throw std::logic_error("CSPLiteral::makeEqual: must not be called");
}
int CSPLiteral::lparseUid(LparseOutputter &) const {
    throw std::logic_error("CSPLiteral::lparseUid: must not be called");
}
CSPLiteral::~CSPLiteral() { }

// }}}

} } // namespace Output Gringo
