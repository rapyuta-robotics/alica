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

#ifndef _GRINGO_GROUND_DEPENDENCY_HH
#define _GRINGO_GROUND_DEPENDENCY_HH

#include <gringo/graph.hh>
#include <gringo/term.hh>
#include <gringo/unique_list.hh>

namespace Gringo { namespace Ground {

// {{{ declaration of Lookup

template <class Occ>
struct Lookup {
    typedef std::unordered_multimap<FWSignature, GTerm*> SigLookup;
    typedef std::unordered_multimap<GTerm*, Occ, value_hash<GTerm*>, value_equal_to<GTerm*>> Occurrences;
    typedef typename Occurrences::iterator iterator;
    //! Adds an occurrence associated with a term.
    //! If there is already an occurrence with a structurally equivalent term,
    //! then the method returns true. Otherwise, the method returns false and
    //! the freshly inserted occurrence is associated with the (representative) 
    //! term already present.
    bool add(GTerm &term, Occ &&occ);
    template <class Callback>
    void match(Value const &x, Callback const &c);
    template <class Callback>
    void unify(GTerm &x, Callback const &c);
    ~Lookup();

private:
    template <class Callback>
    void unify(GTerm &x, SigLookup &y, Callback const &c);

    SigLookup terms;
    SigLookup constTerms;
public:
    Occurrences occs;
};

// }}}
// {{{ declaration of BodyOccurrence

using LocSet = std::set<Location>;
using SigSet = unique_list<FWSignature>;
enum class OccurrenceType { POSITIVELY_STRATIFIED, STRATIFIED, UNSTRATIFIED };
template <class HeadOcc>
struct BodyOccurrence {
    typedef std::vector<std::reference_wrapper<HeadOcc>> DefinedBy;
    virtual UGTerm getRepr() const = 0;
    virtual bool isPositive() const = 0;
    virtual bool isNegative() const = 0;
    virtual void setType(OccurrenceType x) = 0;
    virtual OccurrenceType getType() const = 0;
    virtual DefinedBy &definedBy() = 0;
    virtual void checkDefined(LocSet &done, SigSet const &edb) const = 0;
    virtual ~BodyOccurrence() { }
};

// }}}
// {{{ declaration of Dependency

template <class Stm, class HeadOcc>
struct Dependency {
    struct Node;
    typedef std::vector<Node*> NodeVec;
    typedef std::tuple<BodyOccurrence<HeadOcc>*, NodeVec, bool> Depend;
    typedef std::pair<Node*, typename NodeVec::size_type> Provide;
    typedef Ground::Lookup<Provide> Lookup;
    typedef Graph<Node*> G;
    typedef std::vector<std::pair<std::vector<Stm>,bool>> ComponentVec;
    struct Node {
        Node(Stm &&stm, bool normal) : stm(std::forward<Stm>(stm)), normal(normal) { }

        Stm stm;
        bool normal;
        std::vector<Depend> depend;
        typename G::Node *graphNode = nullptr;
        unsigned negSCC = 0;
        unsigned posSCC = 0;
    };

    Node &add(Stm &&stm, bool normal);
    void depends(Node& n, BodyOccurrence<HeadOcc> &occ, bool forceNegative = false);
    void provides(Node& n, HeadOcc &occ, UGTerm &&term);
    ComponentVec analyze();

    UGTermVec terms;
    Lookup depend;
    std::forward_list<Node> nodes;
    std::vector<std::tuple<Node*, std::reference_wrapper<HeadOcc>, UGTerm>> heads;
};

// }}}

// {{{ definition of Lookup

template <class Occ>
bool Lookup<Occ>::add(GTerm &term, Occ &&x) {
    auto it = occs.find(&term);
    if (it == occs.end()) {
        if (term.eval().first) { constTerms.emplace(term.sig(), &term); }
        else                   { terms.emplace(term.sig(), &term); }
        occs.emplace(&term, std::forward<Occ>(x));
        return true;
    }
    else { 
        occs.emplace_hint(it, it->first, std::forward<Occ>(x));
        return false;
    }
}

template <class Occ>
template <class Callback>
void Lookup<Occ>::match(Value const &x, Callback const &c) {
    switch (x.type()) {
        case Value::FUNC:
        case Value::ID: {
            auto ir(terms.equal_range(x.sig()));
            for (auto it = ir.first; it != ir.second; ++it) {
                if (it->second->match(x)) { 
                    auto rng(occs.equal_range(it->second));
                    assert(rng.first != rng.second);
                    c(rng.first, rng.second);
                }
                it->second->reset();
            }
            GValTerm y(x);
            auto rng(occs.equal_range(&y));
            if (rng.first != rng.second) { c(rng.first, rng.second); }
        }
        default: { }
    }
}

template <class Occ>
template <class Callback>
void Lookup<Occ>::unify(GTerm &x, SigLookup &y, Callback const &c) {
    auto ir(y.equal_range(x.sig()));
    for (auto it = ir.first; it != ir.second; ++it) {
        if (it->second->unify(x)) {
            auto rng(occs.equal_range(it->second));
            assert(rng.first != rng.second);
            c(rng.first, rng.second);
        }
        it->second->reset();
        x.reset();
    }
}

template <class Occ>
template <class Callback>
void Lookup<Occ>::unify(GTerm &x, Callback const &c) {
    auto r = x.eval();
    if (r.first) { match(r.second, c); }
    else {
        unify(x, terms, c);
        unify(x, constTerms, c);
    }
}

template <class Occ>
Lookup<Occ>::~Lookup() { }

// }}}
// {{{ definition of Dependency

template <class Stm, class HeadOcc>
typename Dependency<Stm, HeadOcc>::Node &Dependency<Stm, HeadOcc>::add(Stm &&stm, bool normal) {
    nodes.emplace_front(std::forward<Stm>(stm), normal);
    return nodes.front();
}

template <class Stm, class HeadOcc>
void Dependency<Stm, HeadOcc>::depends(Node& n, BodyOccurrence<HeadOcc> &occ, bool forceNegative) {
    //std::cerr << *n.stm << " depends on " << *occ.getRepr() << (occ.isPositive() ? " pos" : "")  << (forceNegative || occ.isNegative() ? " neg" : "") << std::endl;
    terms.emplace_back(occ.getRepr());
    depend.add(*terms.back(), Provide(&n, n.depend.size()));
    n.depend.emplace_back(&occ, NodeVec(), forceNegative);
    occ.definedBy().clear();
}

template <class Stm, class HeadOcc>
void Dependency<Stm, HeadOcc>::provides(Node& n, HeadOcc& occ, UGTerm &&term) {
    //std::cerr << *n.stm << " provides   " << *term << std::endl;
    heads.emplace_back(&n, occ, std::move(term));
}

template <class Stm, class HeadOcc>
typename Dependency<Stm, HeadOcc>::ComponentVec Dependency<Stm, HeadOcc>::analyze() {
    // initialize nodes
    for (auto &x : heads) {
        auto f = [&x](typename Lookup::iterator begin, typename Lookup::iterator end) -> void {
            for (auto it = begin; it != end; ++it) { 
                auto &dep(it->second.first->depend[it->second.second]);
                std::get<1>(dep).emplace_back(std::get<0>(x));
                std::get<0>(dep)->definedBy().emplace_back(std::get<1>(x));
            }
        };
        depend.unify(*std::get<2>(x), f);
    }
    heads.clear();
    // build dependency graph
    G g;
    for (auto &x : nodes) { x.graphNode = &g.insertNode(&x); }
    for (auto &x : nodes) {
        for (auto &y : x.depend) {
            for (auto &z : std::get<1>(y)) { x.graphNode->insertEdge(*z->graphNode); }
        }
    }
    std::vector<bool> positive;
    ComponentVec components;
    positive.push_back(true);
    for (auto &scc : g.tarjan()) {
        // dependency analysis
        unsigned negSCC = positive.size();
        for (auto &graphNode : scc) { graphNode->data->negSCC = negSCC; }
        positive.push_back(true);
        for (auto &graphNode : scc) {
            positive.back() = positive.back() && graphNode->data->normal;
            for (auto &x : graphNode->data->depend) {
                for (auto &y : std::get<1>(x)) {
                    positive.back() = positive.back() && (y->negSCC != negSCC ? positive[y->negSCC] : std::get<0>(x)->isPositive() && !std::get<2>(x));
                }
            }
        }
        // build positive dependency graph
        G gg;
        for (auto &graphNode : scc) { graphNode->data->graphNode = &gg.insertNode(graphNode->data); }
        for (auto &graphNode : scc) {
            for (auto &x : graphNode->data->depend) {
                if (!std::get<0>(x)->isNegative()) {
                    for (auto &y : std::get<1>(x)) {
                        if (y->negSCC == negSCC) { graphNode->data->graphNode->insertEdge(*y->graphNode); }
                    }
                }
            }
        }
        unsigned posSCC = 0;
        auto posSccs(gg.tarjan());
        for (auto &scc : posSccs) {
            // positive dependency analysis
            for (auto &graphNode : scc) { graphNode->data->posSCC = posSCC; }
            posSCC++;
        }
        posSCC = 0;
        for (auto &scc : posSccs) {
            components.emplace_back();
            for (auto &graphNode : scc) { 
                for (auto &x : graphNode->data->depend) {
                    OccurrenceType t = OccurrenceType::POSITIVELY_STRATIFIED;
                    for (auto &y : std::get<1>(x)) {
                        if (y->negSCC != negSCC) {
                            if (t == OccurrenceType::POSITIVELY_STRATIFIED && !positive[y->negSCC]) { t = OccurrenceType::STRATIFIED; }
                        }
                        else if (y->posSCC < posSCC) {
                            assert(y->negSCC == negSCC);
                            if (t == OccurrenceType::POSITIVELY_STRATIFIED)  { t = OccurrenceType::STRATIFIED; }
                        }
                        else {
                            t = OccurrenceType::UNSTRATIFIED;
                            break;
                        }
                    }
                    std::get<0>(x)->setType(t);
                }
                components.back().second = positive.back();
                components.back().first.emplace_back(std::move(graphNode->data->stm));
            }
            posSCC++;
        }
    }
    return components;
}
// }}}

} } // namespace Ground Gringo

#endif // _GRINGO_GROUND_DEPENDENCY_HH
