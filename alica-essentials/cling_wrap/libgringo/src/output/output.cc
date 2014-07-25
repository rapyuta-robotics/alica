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

#include "gringo/output/output.hh"
#include "gringo/logger.hh"

namespace Gringo { namespace Output {

namespace {

struct DefaultLparseTranslator;

// {{{ declaration of Bound

struct Bound {
    using const_iterator = enum_interval_set<int>::const_iterator;
    using atom_vec = std::vector<std::pair<int, SAuxAtom>>;
    Bound(Value var) 
        : modified(true)
        , var(var) {
        _range.add(std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
    }
    operator Value const & () const { return var; }
    bool init(DefaultLparseTranslator &x);
    int getLower(int coef) const { 
        if (_range.empty()) { return 0; }
        return coef * (coef < 0 ? _range.back() : _range.front()); 
    }
    int getUpper(int coef) const { 
        if (_range.empty()) { return -coef; }
        return coef * (coef < 0 ? _range.front() : _range.back()); 
    }
    void remove(int l, int r) {
        _range.remove(l, r);
        modified = true;
    }
    void add(int l, int r) {
        _range.add(l, r);
        modified = true;
    }
    void clear() {
        _range.clear();
        modified = true;
    }
    void intersect(enum_interval_set<int> &x) {
        _range.intersect(x);
        modified = true;
    }
    const_iterator begin() const { return _range.begin(); }
    const_iterator end() const { return _range.end(); }

    bool                   modified;
    Value                  var;
    
    atom_vec               atoms;
    enum_interval_set<int> _range;
};

// }}}
// {{{ auxiliary functions

bool showSig(OutputPredicates const &outPreds, Signature const &sig, bool csp = false) {
    if (outPreds.empty()) { return true; }
    auto le = [](OutputPredicates::value_type const &x, OutputPredicates::value_type const &y) -> bool { 
        if (std::get<1>(x) != std::get<1>(y)) { return std::get<1>(x) < std::get<1>(y); }
        return std::get<2>(x) < std::get<2>(y);
    };
    static Location loc("",1,1,"",1,1);
    return std::binary_search(outPreds.begin(), outPreds.end(), OutputPredicates::value_type(loc, sig, csp), le);
}
bool showBound(OutputPredicates const &outPreds, Bound const &bound) {
    return outPreds.empty() || ((bound.var.type() == Value::FUNC || bound.var.type() == Value::ID) && showSig(outPreds, *bound.var.sig(), true));
}

// }}}
// {{{ declaration of LinearConstraint

struct LinearConstraint {
    struct State {
        using Coef = CoefVarVec::value_type;
        State(Bound &bound, Coef &coef) 
            : bound(bound)
            , coef(coef.first) { reset(); }
        int upper()  { return bound.getUpper(coef); }
        int lower()  { return bound.getLower(coef); }
        bool valid() { return atom != (coef < 0 ? bound.atoms.begin() : bound.atoms.end()); }
        void next(int total, int &ret) {
            if (coef < 0) {
                --atom;
                --current;
                auto it = current;
                --it;
                ret = total + coef * (*it);
            }
            else {
                ++atom;
                ++current;
                ret = total + coef * (*current);
            }
        }
        void reset() {
            current = coef < 0 ? bound.end() : bound.begin();
            atom    = coef < 0 ? bound.atoms.end() : bound.atoms.begin();
        }
        Bound                                 &bound;
        enum_interval_set<int>::const_iterator current;
        Bound::atom_vec::iterator              atom;
        int                                    coef;
    };
    using StateVec = std::vector<State>;
    struct Generate {
        Generate(LinearConstraint &cons, DefaultLparseTranslator &trans)
            : cons(cons)
            , trans(trans) { }
        bool init();
        int getBound() { return cons.bound; }
        void generate(StateVec::iterator it, int current, int remainder);
        StateVec               states;
        LinearConstraint            &cons;
        DefaultLparseTranslator &trans;
        SAuxAtomVec            aux;
    };
    LinearConstraint(SAuxAtom atom, CoefVarVec &&coefs, int bound)
        : atom(atom)
        , coefs(std::move(coefs))
        , bound(bound) { }
    bool encode(DefaultLparseTranslator &x);
    SAuxAtom   atom;
    CoefVarVec coefs;
    int        bound;
};

// }}}
// {{{ declaration of DisjointConstraint

struct DisjointConstraint {
    DisjointConstraint(SAuxAtom atom, DisjointCons &&cons)
        : atom(atom)
        , cons(std::move(cons)) { }
    bool encode(DefaultLparseTranslator &x);
    SAuxAtom     atom;
    DisjointCons cons;
};

// }}}
// {{{ declaration of DefaultLparseTranslator

struct DefaultLparseTranslator : LparseTranslator {
    using StmPrinter      = std::function<void (Statement const &)>;
    using MinimizeList    = std::vector<std::pair<FWValVec, ULitVec>>;
    using BoundMap        = unique_list<Bound, identity<Value>>;
    using ConstraintVec   = std::vector<LinearConstraint>;
    using DisjointConsVec = std::vector<DisjointConstraint>;

    DefaultLparseTranslator(StmPrinter const &printer)
        : printer(printer) { }
    BoundMap::value_type &addBound(Value x) {
        auto it = boundMap.find(x);
        return it != boundMap.end() ? *it : *boundMap.emplace_back(x).first;
    }
    static int min() { return std::numeric_limits<int>::min(); }
    static int max() { return std::numeric_limits<int>::max(); }
    virtual void addLowerBound(Value x, int bound) {
        auto &y = addBound(x);
        y.remove(std::numeric_limits<int>::min(), bound);
    }
    virtual void addUpperBound(Value x, int bound) {
        auto &y = addBound(x);
        y.remove(bound+1, std::numeric_limits<int>::max());
    }
    virtual void addBounds(Value value, std::vector<CSPBound> bounds) {
        std::map<Value, enum_interval_set<int>> boundUnion;
        for (auto &x : bounds) {
            boundUnion[value].add(x.first, x.second+1);
        }
        for (auto &x : boundUnion) {
            auto &z = addBound(x.first);
            z.intersect(x.second);
        }
    }
    virtual void addLinearConstraint(SAuxAtom head, CoefVarVec &&vars, int bound) {
        for (auto &x : vars) { addBound(x.second); }
        constraints.emplace_back(head, std::move(vars), bound);
    }
    virtual void addDisjointConstraint(SAuxAtom head, DisjointCons &&elem) {
        for (auto &x : elem) { 
            for (auto &y : x.second) {
                for (auto z : y.value) { addBound(z.second); }
            }
        }
        disjointCons.emplace_back(head, std::move(elem));
    }
    virtual unsigned auxAtom() {
        return ++auxUid;
    }
    virtual void operator()(Statement &x) { printer(x); }
    virtual void addMinimize(MinimizeList &&x) {
        minimizeChanged_ = minimizeChanged_ || !x.empty();
        std::move(x.begin(), x.end(), std::back_inserter(minimize));
    }
    virtual bool minimizeChanged() const {
        return minimizeChanged_;
    }
    virtual void translate() {
        for (auto &x : boundMap) { 
            if (!x.init(*this)) { return; }
        }
        for (auto &x : disjointCons) { x.encode(*this); }
        for (auto &x : constraints)  { x.encode(*this); }
        disjointCons.clear();
        constraints.clear();
        if (minimizeChanged_) {
            translateMinimize();
            minimizeChanged_ = false;
        }
    }
    virtual void outputSymbols(LparseOutputter &out, PredDomMap const &domains, OutputPredicates const &outPreds) {
        std::vector<std::tuple<unsigned, Value, int>> symtab;
        auto show = [&](Bound const &bound, AtomState *cond) -> void {
            std::string const *name = nullptr;
            if (bound.var.type() == Value::FUNC || bound.var.type() == Value::ID) { 
                name = &*bound.var.name();
            }
            if ((!name || name->empty() || name->front() != '#')) {
                auto assign = [&](int i, SAuxAtom a, SAuxAtom b) {
                    LparseOutputter::LitVec body;
                    if (a) { body.emplace_back(a->lparseUid(out)); }
                    if (b) { body.emplace_back(-b->lparseUid(out)); }
                    if (cond && cond->uid() && !cond->fact(false)) { body.emplace_back(cond->uid()); }
                    unsigned head = out.newUid();
                    out.printBasicRule(head, body);
                    symtab.emplace_back(head, bound.var, i);
                };
                auto it = bound.begin();
                for (auto jt = bound.atoms.begin() + 1; jt != bound.atoms.end(); ++jt) { assign(*it++, jt->second, (jt-1)->second); }
                assign(*it++, nullptr, bound.atoms.back().second);
            }
        };
        // show csp varibles
        {
            auto it = incBoundOffset, ie = boundMap.end();
            if (it == ie) { it = boundMap.begin(); }
            else          { ++it; }
            for (; it != ie; ++it) {
                if (it->var.type() == Value::FUNC || it->var.type() == Value::ID) { seenSigs.emplace(it->var.sig()); }
                if (showBound(outPreds, *it)) { show(*it, nullptr); }
                incBoundOffset = it;
            }
        }
        // show explicitely shown csp variables (if not shown above)
        auto it(domains.find(Signature("#show", 2)));
        if (it != domains.end()) {
            for (auto jt = it->second.exports.begin() + it->second.exports.incOffset, je = it->second.exports.end(); jt != je; ++jt) {
                if ((jt->get().first.args().begin()+1)->num() == 1) {
                    Value val = *(jt->get().first.args().begin());
                    auto bound = boundMap.find(val);
                    if (bound != boundMap.end()) {
                        if (!showBound(outPreds, *bound)) { show(*bound, &jt->get().second); }
                    }
                    else {
                        GRINGO_REPORT(W_TERM_UNDEFINED) 
                            << "warning: trying to show constraint variable that does not occur in program:\n"
                            << "  $" << val << "\n";
                    }
                }
            }
        }
        // check for signatures that did not occur in the program
        for (auto &x : outPreds) {
            if (std::get<1>(x) != Signature("", 0) && std::get<2>(x)) {
                auto it(seenSigs.find(std::get<1>(x)));
                if (it == seenSigs.end()) {
                    GRINGO_REPORT(W_TERM_UNDEFINED) 
                        << std::get<0>(x) << ": warning: no matching occurrence for signature:\n"
                        << "  $" << *std::get<1>(x) << "\n";
                    seenSigs.emplace(std::get<1>(x));
                }
            }
        }
        out.finishRules();
        for (auto &y : symtab) {
            std::ostringstream oss;
            oss 
                << std::get<1>(y)
                << "="
                << std::get<2>(y);
            out.printSymbol(std::get<0>(y), oss.str());
        }
    }
    void atoms(int atomset, std::function<bool(unsigned)> const &isTrue, ValVec &atoms, PredDomMap const &domains, OutputPredicates const &outPreds) {
        auto showCsp = [&isTrue, &atoms](Bound const &bound) {
            assert(!bound.atoms.empty());
            int prev = bound.atoms.front().first;
            for (auto it = bound.atoms.begin()+1; it != bound.atoms.end() && !isTrue(it->second->uid); ++it);
            atoms.emplace_back(Value("$", {bound.var, prev}));
        };
        if (atomset & (Model::CSP | Model::SHOWN)) {
            for (auto &x : boundMap) {
                if (atomset & Model::CSP || (atomset & Model::SHOWN && showBound(outPreds, x))) { showCsp(x); }
            }
        }
        if (atomset & Model::SHOWN) {
            auto it(domains.find(Signature("#show", 2)));
            if (it != domains.end()) {
                for (auto &y: it->second.domain) {
                    if (y.first.args().back() == Value(1) && y.second.defined() && y.second.hasUid() && isTrue(y.second.uid())) {
                        auto bound = boundMap.find(*y.first.args().begin());
                        if (bound != boundMap.end() && !showBound(outPreds, *bound)) { showCsp(*bound); }
                    }
                }
            }
        }
    }
    void translateMinimize();

    virtual ~DefaultLparseTranslator() { }

    BoundMap           boundMap;
    ConstraintVec      constraints;
    DisjointConsVec    disjointCons;
    MinimizeList       minimize;
    StmPrinter         printer;
    unsigned           auxUid = 0;
    bool               minimizeChanged_ = false;
    std::set<FWSignature> seenSigs;
    BoundMap::iterator incBoundOffset;
};

// }}}

// {{{ definition of DefaultLparseTranslator

void DefaultLparseTranslator::translateMinimize() {
    // TODO: should detect if there are changes in the minimize constraint
    // priority -> tuple -> [[lit]]
    using GroupByPriority = std::map<Value, unique_list<std::pair<FWValVec, std::vector<ULitVec>>, extract_first<FWValVec>>>;
    GroupByPriority groupBy;
    for (auto &y : minimize) {
        auto &ret(groupBy.insert(GroupByPriority::value_type(*(y.first.begin() + 1), GroupByPriority::mapped_type())).first->second);
        ret.emplace_back(
            std::piecewise_construct,
            std::forward_as_tuple(y.first),
            std::forward_as_tuple()).first->second.emplace_back(get_clone(y.second));
    }
    for (auto &y : groupBy) {
        ULitWeightVec weightLits;
        for (auto &conds : y.second) {
            ULitVec condLits;
            int weight((*conds.first.begin()).num());
            for (auto &cond : conds.second) {
                if (cond.size() == 1) {
                    ULit lit = std::move(cond.front());
                    condLits.emplace_back(std::move(lit));
                }
                else {
                    // TODO: a function that creates an inverted literal would be nice!!!
                    SAuxAtom atom(std::make_shared<AuxAtom>(auxAtom()));
                    printer(LparseRule(atom, std::move(cond)));
                    condLits.emplace_back(make_unique<AuxLiteral>(atom, false));
                }
            }
            if (condLits.size() == 1) {
                if (weight < 0) {
                    SAuxAtom atom(std::make_shared<AuxAtom>(auxAtom()));
                    printer(LparseRule(atom, nullptr, std::move(condLits.front())));
                    weightLits.emplace_back(std::move(make_unique<AuxLiteral>(atom, true)), std::abs(weight));
                }
                else {
                    weightLits.emplace_back(std::move(condLits.front()), std::abs(weight));
                }
            }
            else {
                SAuxAtom atom(std::make_shared<AuxAtom>(auxAtom()));
                for (auto &cond : condLits) { printer(LparseRule(atom, std::move(cond), nullptr)); }
                weightLits.emplace_back(make_unique<AuxLiteral>(atom, weight < 0), std::abs(weight));
            }
        }
        printer(LparseMinimize(y.first, std::move(weightLits)));
    }
}
// }}}
// {{{ definition of Bound

bool Bound::init(DefaultLparseTranslator &x) {
    if (modified) {
        modified = false;
        if (_range.empty()) { x.printer(LparseRule(SAuxAtomVec(), ULitVec(), false)); }
        else {
            if (_range.front() == std::numeric_limits<int>::min() || _range.back()+1 == std::numeric_limits<int>::max()) {
                if      (_range.front()  != std::numeric_limits<int>::min()) { _range.remove(_range.front()+1, std::numeric_limits<int>::max()); }
                else if (_range.back()+1 != std::numeric_limits<int>::max()) { _range.remove(std::numeric_limits<int>::min(), _range.back()); }
                else                                                        { _range.clear(), _range.add(0, 1); }
                GRINGO_REPORT(W_TERM_UNDEFINED)
                    << "warning: unbounded constraint variable, domain is set to [" << _range.front() << "," << _range.back() << "]:\n"
                    << "  " << var << "\n";
                    ;
            }
            if (atoms.empty()) {
                auto assign = [&](SAuxAtom a, SAuxAtom b) {
                    if (b) {
                        SAuxAtomVec head;
                        ULitVec body;
                        head.emplace_back(b);
                        if (a) { body.emplace_back(make_unique<AuxLiteral>(a, false)); }
                        x.printer(LparseRule(std::move(head), std::move(body), true));
                    }
                };
                for (auto y : _range) { 
                    if (y == _range.front()) { atoms.emplace_back(y, nullptr); } 
                    else                     { atoms.emplace_back(y, std::make_shared<AuxAtom>(x.auxAtom())); }
                }
                for (auto jt = atoms.begin() + 1; jt != atoms.end(); ++jt) { assign(jt->second, (jt-1)->second); }
                assign(nullptr, atoms.back().second);
            }
            else { // incremental update of bounds
                atom_vec next;
                int l = _range.front(), r = _range.back();
                for (auto jt = atoms.begin() + 1; jt != atoms.end(); ++jt) {
                    int w = (jt - 1)->first;
                    if (w < l)                           { x.printer(LparseRule(make_unique<AuxLiteral>(jt->second, false), nullptr)); }
                    else if (w >= r)                     { 
                        x.printer(LparseRule(make_unique<AuxLiteral>(jt->second, true), nullptr));
                        if (w == r) { next.emplace_back(*(jt - 1)); }
                    }
                    else if (!_range.contains(w, w + 1)) { x.printer(LparseRule(make_unique<AuxLiteral>((jt-1)->second, true), make_unique<AuxLiteral>(jt->second, false))); }
                    else                                 { next.emplace_back(*(jt - 1)); }
                }
                if (atoms.back().first <= r) { next.emplace_back(atoms.back()); }
                next.front().second = nullptr;
                atoms = std::move(next);
            }
        }
    }
    return !_range.empty();
}

// }}}
// {{{ definition of LinearConstraint

int depth = 0;
void LinearConstraint::Generate::generate(StateVec::iterator it, int current, int remainder) {
    depth++;
    if (current + remainder > getBound()) {
        if (it == states.end()) {
            assert(remainder == 0);
            aux.emplace_back(std::make_shared<AuxAtom>(trans.auxAtom()));
            for (auto &x : states) {
                if (x.atom != x.bound.atoms.end() && x.atom->second) { 
                    trans.printer(LparseRule(aux.back(), make_unique<AuxLiteral>(x.atom->second, x.coef < 0), nullptr));
                }
            }
        }
        else {
            remainder -= it->upper() - it->lower();
            int total = current - it->lower();
            for (it->reset(); it->valid(); it->next(total, current)) { 
                generate(it+1, current, remainder);
                if (current > getBound()) { break; }
            }
        }
    }
    depth--;
}

bool LinearConstraint::Generate::init() {
    int current   = 0;
    int remainder = 0;
    for (auto &y : cons.coefs) {
        states.emplace_back(*trans.boundMap.find(y.second), y);
        current   += states.back().lower();
        remainder += states.back().upper() - states.back().lower();
    }
    if (current <= getBound()) {
        generate(states.begin(), current, remainder); 
        ULitVec body;
        for (auto &x : aux) { body.emplace_back(make_unique<AuxLiteral>(x, false)); }
        trans.printer(LparseRule(cons.atom, std::move(body)));
    }
    return current <= cons.bound;
}

bool LinearConstraint::encode(DefaultLparseTranslator &x) {
    Generate gen(*this, x);
    return gen.init();
}

// }}}
// {{{ definition of DisjointConstraint

bool DisjointConstraint::encode(DefaultLparseTranslator &x) {
    std::set<int> values;
    std::vector<std::map<int, SAuxAtom>> layers;
    for (auto &elem : cons) {
        layers.emplace_back();
        auto &current = layers.back();
        auto encodeSingle = [&current, &x, &values](ULitVec const &cond, int coef, Value var, int fixed) -> void {
            auto &bound = *x.boundMap.find(var);
            auto it = bound.atoms.begin() + 1;
            for(auto i : bound) {
                auto &aux = current[i*coef + fixed];
                if (!aux) { aux = std::make_shared<AuxAtom>(x.auxAtom()); }
                ULitVec lits = get_clone(cond);
                if ((it-1)->second)          { lits.emplace_back(make_unique<AuxLiteral>((it-1)->second, true)); }
                if (it != bound.atoms.end()) { lits.emplace_back(make_unique<AuxLiteral>(it->second, false)); }
                x.printer(LparseRule(aux, std::move(lits)));
                values.insert(i*coef + fixed);
                ++it;
            }
        };
        for (auto &condVal : elem.second) {
            switch(condVal.value.size()) {
                case 0: {
                    auto &aux = current[condVal.fixed];
                    if (!aux) { aux = std::make_shared<AuxAtom>(x.auxAtom()); }
                    x.printer(LparseRule(aux, std::move(condVal.lits)));
                    values.insert(condVal.fixed);
                    break;
                }
                case 1: {
                    encodeSingle(condVal.lits, condVal.value.front().first, condVal.value.front().second, condVal.fixed);
                    break;
                }
                default: {
                    // create a bound possibly with holes
                    auto &b = *x.boundMap.emplace_back(Value("#aux" + std::to_string(x.auxAtom()))).first;
                    b.clear();
                    std::set<int> values;
                    values.emplace(0);
                    for (auto &mul : condVal.value) {
                        std::set<int> nextValues;
                        for (auto i : *x.boundMap.find(mul.second)) {
                            for (auto j : values) { nextValues.emplace(j + i * mul.first); }
                        }
                        values = std::move(nextValues);
                    }
                    for (auto i : values) { b.add(i, i+1); }
                    b.init(x);

                    // create a new variable with the respective bound
                    // b = value + fixed
                    // b - value = fixed
                    // aux :- b - value <=  fixed - 1
                    // aux :- value - b <= -fixed - 1
                    // :- aux.
                    SAuxAtom aux(std::make_shared<AuxAtom>(x.auxAtom()));
                    condVal.value.emplace_back(-1, b.var);
                    x.addLinearConstraint(aux, get_clone(condVal.value), -condVal.fixed-1);
                    for (auto &add : condVal.value) { add.first *= -1; }
                    x.addLinearConstraint(aux, get_clone(condVal.value), condVal.fixed-1);
                    x.printer(LparseRule(make_unique<AuxLiteral>(aux, false), nullptr));

                    // then proceed as in case one
                    encodeSingle(condVal.lits, 1, b.var, 0);
                    break;
                }
            }
        }
    }
    ULitVec checks;
    for (auto &value : values) {
        WeightRule::ULitBoundVec card;
        for (auto &layer : layers) {
            auto it = layer.find(value);
            if (it != layer.end()) {
                card.emplace_back(make_unique<AuxLiteral>(it->second, false), 1);
            }
        }
        SAuxAtom aux = std::make_shared<AuxAtom>(x.auxAtom());
        checks.emplace_back(make_unique<AuxLiteral>(aux, true));
        x.printer(WeightRule(aux, 2, std::move(card)));
    }
    x.printer(LparseRule(atom, std::move(checks)));
    return true;
}

// }}}

// {{{ definition of LparseHandler

struct LparseHandler : StmHandler {
    LparseHandler(LparseOutputter &out, LparseDebug debug) 
        : trans([&out, debug](Statement const &x) {
            if ((unsigned)debug & (unsigned)LparseDebug::LPARSE) { std::cerr << "%%"; x.printPlain(std::cerr); }
            x.printLparse(out);
        }) 
        , out(out)
        , debug(debug) { }
    virtual void operator()(Statement &x) { 
        if ((unsigned)debug & (unsigned)LparseDebug::PLAIN) { std::cerr << "%"; x.printPlain(std::cerr); }
        x.toLparse(trans);
    }
    virtual void incremental() { out.incremental(); }
    virtual void operator()(PredicateDomain::element_type &head, ExternalType type) {
        if (!head.second.hasUid()) { head.second.uid(out.newUid()); }
        out.printExternal(head.second.uid(), type);
    }
    virtual void finish(PredDomMap &domains, OutputPredicates &outPreds) {
        out.disposeMinimize() = trans.minimizeChanged();
        trans.translate(); 
        trans.outputSymbols(out, domains, outPreds);
        if (!outPreds.empty()) { 
            for (auto &x : outPreds) {
                if (!std::get<2>(x)) {
                    auto it(domains.find(std::get<1>(x)));
                    if (it != domains.end()) {
                        for (auto jt = it->second.exports.begin() + it->second.exports.incOffset, je = it->second.exports.end(); jt != je; ++jt) {
                            if (!jt->get().second.hasUid()) { jt->get().second.uid(out.newUid()); }
                            out.printSymbol(jt->get().second.uid(), jt->get().first);
                        }
                    }
                }
            }
        }
        else {
            for (auto &x : domains) {
                std::string const &name(*(*x.first).name());
                if ((name.empty() || name.front() != '#') && outPreds.empty()) {
                    for (auto jt = x.second.exports.begin() + x.second.exports.incOffset, je = x.second.exports.end(); jt != je; ++jt) {
                        if (!jt->get().second.hasUid()) { jt->get().second.uid(out.newUid()); }
                        out.printSymbol(jt->get().second.uid(), jt->get().first);
                    }
                }
            }
        }
        // Note: show domains are cleared anyway
        auto it(domains.find(Signature("#show", 2)));
        if (it != domains.end()) {
            for (auto jt = it->second.exports.begin() + it->second.exports.incOffset, je = it->second.exports.end(); jt != je; ++jt) {
                if ((jt->get().first.args().begin()+1)->num() == 0) {
                    if (!jt->get().second.hasUid()) { jt->get().second.uid(out.newUid()); }
                    out.printSymbol(jt->get().second.uid(), *jt->get().first.args().begin());
                }
            }
        }
        out.finishSymbols();
    }
    virtual void atoms(int atomset, std::function<bool(unsigned)> const &isTrue, ValVec &atoms, PredDomMap const &domains, OutputPredicates const &outPreds) {
        if (atomset & (Model::ATOMS | Model::SHOWN)) {
            for (auto &x : domains) {
                Signature sig = *x.first;
                std::string const &name = *sig.name();
                if ((atomset & Model::ATOMS && !name.empty() && name.front() != '#') || (atomset & Model::SHOWN && showSig(outPreds, *x.first))) {
                    for (auto &y: x.second.domain) {
                        if (y.second.defined() && y.second.hasUid() && isTrue(y.second.uid())) { atoms.emplace_back(y.first); }
                    }
                }
            }
        }
        if ((atomset & (Model::TERMS | Model::SHOWN))) {
            auto it(domains.find(Signature("#show", 2)));
            if (it != domains.end()) {
                for (auto &y: it->second.domain) {
                    if (y.first.args().back() == Value(0) && y.second.defined() && y.second.hasUid() && isTrue(y.second.uid())) { atoms.emplace_back(y.first.args().front()); }
                }
            }
        }
        trans.atoms(atomset, isTrue, atoms, domains, outPreds);
    }
    virtual ~LparseHandler() { }

    DefaultLparseTranslator trans;
    LparseOutputter &out;
    LparseDebug debug;
};

// }}}
// {{{ definition of LparsePlainHandler

struct LparsePlainHandler : StmHandler {
    LparsePlainHandler(std::ostream &out)
        : trans([&out](Statement const &x)                { x.printPlain(out); })
        , out(out) { }
    virtual void operator()(Statement &x)                 { x.toLparse(trans); }
    virtual void operator()(PredicateDomain::element_type &head, ExternalType type) {
        switch (type) {
            case ExternalType::E_FALSE: { out << "#external " << head.first << ".\n"; break; }
            case ExternalType::E_TRUE:  { out << "#external " << head.first << "=true.\n"; break; }
            case ExternalType::E_FREE:  { out << "#external " << head.first << "=free.\n"; break; }
        }
    }
    virtual void finish(PredDomMap &, OutputPredicates &) { trans.translate(); }
    virtual void atoms(int, std::function<bool(unsigned)> const &, ValVec &, PredDomMap const &, OutputPredicates const &) { }
    virtual ~LparsePlainHandler()                         { }

    DefaultLparseTranslator trans;
    std::ostream           &out;
};

// }}}
// {{{ definition of PlainHandler

struct PlainHandler : StmHandler {
    PlainHandler(std::ostream &out) : out(out)            { }
    virtual ~PlainHandler()                               { }
    virtual void operator()(Statement &x)                 { x.printPlain(out); }
    virtual void operator()(PredicateDomain::element_type &head, ExternalType type) {
        switch (type) {
            case ExternalType::E_FALSE: { out << "#external " << head.first << ".\n"; break; }
            case ExternalType::E_TRUE:  { out << "#external " << head.first << "=true.\n"; break; }
            case ExternalType::E_FREE:  { out << "#external " << head.first << "=free.\n"; break; }
        }
    }
    virtual void finish(PredDomMap &, OutputPredicates &outPreds) { 
        for (auto &x : outPreds) { 
            if (std::get<1>(x) != Signature("", 0)) { std::cout << "#show " << (std::get<2>(x) ? "$" : "") << *std::get<1>(x) << ".\n"; }
            else                                    { std::cout << "#show.\n"; }
        }
    }
    virtual void atoms(int, std::function<bool(unsigned)> const &, ValVec &, PredDomMap const &, OutputPredicates const &) { }
    std::ostream &out;
};

// }}}

} // namespace

// {{{ definition of PlainLparseOutputter

PlainLparseOutputter::PlainLparseOutputter(std::ostream &out) : out(out) { }
void PlainLparseOutputter::incremental() {
    out << "90 0\n";
}
void PlainLparseOutputter::printBasicRule(unsigned head, LitVec const &body) {
    out << "1 " << head << " " << body.size();
    unsigned neg(0);
    for (auto &x : body) { neg+= x < 0; }
    out << " " << neg;
    for (auto &x : body) { if (x < 0) { out << " " << -x; } }
    for (auto &x : body) { if (x > 0) { out << " " << +x; } }
    out << "\n";
}
void PlainLparseOutputter::printChoiceRule(AtomVec const &head, LitVec const &body) {
    out << "3 " << head.size();
    for (auto &x : head) { out << " " << x; }
    out << " " << body.size();
    unsigned neg(0);
    for (auto &x : body) { neg+= x < 0; }
    out << " " << neg;
    for (auto &x : body) { if (x < 0) { out << " " << -x; } }
    for (auto &x : body) { if (x > 0) { out << " " << +x; } }
    out << "\n";
}
void PlainLparseOutputter::printCardinalityRule(unsigned head, unsigned lower, LitVec const &body) {
    out << "2 " << head << " " << body.size();
    unsigned neg(0);
    for (auto &x : body) { neg+= x < 0; }
    out << " " << neg << " " << lower;
    for (auto &x : body) { if (x < 0) { out << " " << -x; } }
    for (auto &x : body) { if (x > 0) { out << " " << +x; } }
    out << "\n";
}
void PlainLparseOutputter::printWeightRule(unsigned head, unsigned lower, LitWeightVec const &body) {
    out << "5 " << head << " " << lower << " " << body.size();
    unsigned neg(0);
    for (auto &x : body) { neg+= x.first < 0; }
    out << " " << neg;
    for (auto &x : body) { if (x.first < 0) { out << " " << -x.first; } }
    for (auto &x : body) { if (x.first > 0) { out << " " << +x.first; } }
    for (auto &x : body) { if (x.first < 0) { out << " " << x.second; } }
    for (auto &x : body) { if (x.first > 0) { out << " " << x.second; } }
    out << "\n";
}
void PlainLparseOutputter::printMinimize(LitWeightVec const &body) {
    out << "6 0 " << body.size();
    unsigned neg(0);
    for (auto &x : body) { neg+= x.first < 0; }
    out << " " << neg;
    for (auto &x : body) { if (x.first < 0) { out << " " << -x.first; } }
    for (auto &x : body) { if (x.first > 0) { out << " " << +x.first; } }
    for (auto &x : body) { if (x.first < 0) { out << " " << x.second; } }
    for (auto &x : body) { if (x.first > 0) { out << " " << x.second; } }
    out << "\n";
}
void PlainLparseOutputter::printDisjunctiveRule(AtomVec const &head, LitVec const &body) {
    out << "8 " << head.size();
    for (auto &x : head) { out << " " << x; }
    out << " " << body.size();
    unsigned neg(0);
    for (auto &x : body) { neg+= x < 0; }
    out << " " << neg;
    for (auto &x : body) { if (x < 0) { out << " " << -x; } }
    for (auto &x : body) { if (x > 0) { out << " " << +x; } }
    out << "\n";
}
void PlainLparseOutputter::printExternal(unsigned atomUid, ExternalType type) { 
    switch (type) {
        case ExternalType::E_FALSE: { out << "91 " << atomUid << " 0\n"; break; }
        case ExternalType::E_TRUE:  { out << "91 " << atomUid << " 1\n"; break; }
        case ExternalType::E_FREE:  { out << "92 " << atomUid << "\n"; break; }
    }
}
unsigned PlainLparseOutputter::falseUid()                                   { return 1; }
unsigned PlainLparseOutputter::newUid()                                     { return uids++; }
void PlainLparseOutputter::finishRules()                                    { out << "0\n"; }
void PlainLparseOutputter::printSymbol(unsigned atomUid, Value v)           { out << atomUid << " " << v << "\n"; }
void PlainLparseOutputter::finishSymbols()                                  { out << "0\nB+\n0\nB-\n" << falseUid() << "\n0\n1\n"; }
PlainLparseOutputter::~PlainLparseOutputter()                               { }

// }}}
// {{{ definition of OutputBase

OutputBase::OutputBase(OutputPredicates &&outPreds, std::ostream &out, bool lparse)
    : handler(lparse ? UStmHandler(new LparsePlainHandler(out)) : UStmHandler(new PlainHandler(out))) 
    , outPreds(std::move(outPreds)) { }
OutputBase::OutputBase(OutputPredicates &&outPreds, LparseOutputter &out, LparseDebug debug) 
    : handler(make_unique<LparseHandler>(out, debug))
    , outPreds(std::move(outPreds)) { }
void OutputBase::output(Value const &val) {
    auto it(domains.find(val.sig()));
    assert(it != domains.end());
    auto ret(it->second.insert(val, true));
    if (!std::get<2>(ret) || !std::get<1>(ret)) {
        tempRule.head = std::get<0>(ret);
        tempRule.body.clear();
        (*handler)(tempRule);
    }
}
void OutputBase::incremental() {
    handler->incremental();
}
void OutputBase::external(PredicateDomain::element_type &head, ExternalType type) {
    (*handler)(head, type);
}
void OutputBase::output(UStm &&x) {
    if (!x->isIncomplete()) { (*handler)(*x); }
    else { stms.emplace_back(std::move(x)); }
}
void OutputBase::output(Statement &x) {
    if (!x.isIncomplete()) { (*handler)(x); }
    else { stms.emplace_back(x.clone()); }
}
void OutputBase::flush() {
    for (auto &x : stms) { (*handler)(*x); }
    stms.clear();
}
void OutputBase::finish() { 
    handler->finish(domains, outPreds);
} 
void OutputBase::checkOutPreds() {
    auto le = [](OutputPredicates::value_type const &x, OutputPredicates::value_type const &y) -> bool { 
        if (std::get<1>(x) != std::get<1>(y)) { return std::get<1>(x) < std::get<1>(y); }
        return std::get<2>(x) < std::get<2>(y);
    };
    auto eq = [](OutputPredicates::value_type const &x, OutputPredicates::value_type const &y) { 
        return std::get<1>(x) == std::get<1>(y) && std::get<2>(x) == std::get<2>(y); 
    };
    std::sort(outPreds.begin(), outPreds.end(), le);
    outPreds.erase(std::unique(outPreds.begin(), outPreds.end(), eq), outPreds.end());
    for (auto &x : outPreds) {
        if (std::get<1>(x) != Signature("", 0) && !std::get<2>(x)) {
            auto it(domains.find(std::get<1>(x)));
            if (it == domains.end()) {
                GRINGO_REPORT(W_ATOM_UNDEFINED) 
                    << std::get<0>(x) << ": warning: no matching occurrence for signature:\n"
                    << "  " << *std::get<1>(x) << "\n";
            }
        }
    }
}
ValVec OutputBase::atoms(int atomset, std::function<bool(unsigned)> const &isTrue) const {
    Gringo::ValVec ret;
    handler->atoms(atomset, isTrue, ret, domains, outPreds);
    return ret;
}
Gringo::AtomState const *OutputBase::find(Gringo::Value val) const {
    if (val.type() == Gringo::Value::ID || val.type() == Gringo::Value::FUNC) {
        auto it = domains.find(val.sig());
        if (it != domains.end()) {
            auto jt = it->second.domain.find(val);
            if (jt != it->second.domain.end() && jt->second.defined()) {
                return &jt->second;
            }
        }
    }
    return nullptr;
}
PredicateDomain::element_type *OutputBase::find2(Gringo::Value val) {
    if (val.type() == Gringo::Value::ID || val.type() == Gringo::Value::FUNC) {
        auto it = domains.find(val.sig());
        if (it != domains.end()) {
            auto jt = it->second.domain.find(val);
            if (jt != it->second.domain.end() && jt->second.defined()) {
                return &*jt;
            }
        }
    }
    return nullptr;
}

// }}}

} } // namespace Output Gringo
