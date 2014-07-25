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

#ifndef _GRINGO_GROUND_BINDERS_HH
#define _GRINGO_GROUND_BINDERS_HH

#include <gringo/domain.hh>
#include <gringo/ground/instantiation.hh>

namespace Gringo { namespace Ground {

// {{{ definition of PosBinder

template <class Index, class... LookupArgs>
struct PosBinder : Binder {
    using IndexType = typename std::remove_reference<Index>::type;
    using Match     = typename IndexType::element_type*;
    using MatchRng  = typename IndexType::element_range;
    using Lookup    = std::tuple<Index, LookupArgs...>;
    PosBinder(UTerm &&repr, Match &result, Index &&index, BinderType type, LookupArgs&&... args) 
        : repr(std::move(repr))
        , result(result)
        , index(std::forward<Index>(index), std::forward<LookupArgs>(args)...)
        , type(type) { }

    template <       int... I> struct lookup;
    template <int N, int... I> struct lookup<N, I...> : lookup<N-1, N, I...> { };
    template <       int... I> struct lookup<0, I...> {
        MatchRng operator()(std::tuple<Index, LookupArgs...> &index, BinderType type) { 
            return std::get<0>(index).lookup(std::get<I>(index)..., type);
        }
    };

    virtual IndexUpdater *getUpdater()          { return &std::get<0>(index); }
    virtual void match()                        { current = lookup<sizeof...(LookupArgs)>()(index, type); }
    virtual bool next()                         { return (result = current.next(*repr, std::get<0>(index))); }
    virtual void print(std::ostream &out) const { out << *repr << "@" << type; }
    virtual ~PosBinder()                        { }
    
    UTerm      repr; // problematic 
    Match     &result;
    Lookup     index;
    MatchRng   current;
    BinderType type;
};

// }}}
// {{{ definition of Matcher

template <class Element>
struct Matcher : Binder {
    using DomainType = AbstractDomain<Element>;
    using Match      = typename DomainType::element_type*;

    Matcher(Match &result, DomainType &domain, Term const &repr, RECNAF naf)
        : result(result)
        , domain(domain)
        , repr(repr)
        , naf(naf) { }
    virtual IndexUpdater *getUpdater() { return nullptr; }
    virtual void match() {
        result = domain.lookup(repr, naf);
        firstMatch = result;
    }
    virtual bool next() {
        bool ret = firstMatch;
        firstMatch = false;
        return ret;
    }
    virtual void print(std::ostream &out) const { out << naf << repr << "@ALL"; }
    virtual ~Matcher() { }

    Match      &result;
    DomainType &domain;
    Term const &repr;
    RECNAF      naf;
    bool        firstMatch;
};

// }}}
// {{{ definition of PosMatcher

template <class Element>
struct PosMatcher : Binder, IndexUpdater {
    using DomainType = AbstractDomain<Element>;
    using Match = typename DomainType::element_type*;

    PosMatcher(Match &result, DomainType &domain, UTerm &&repr, BinderType type)
        : result(result)
        , domain(domain)
        , repr(std::move(repr))
        , type(type) { }
    virtual IndexUpdater *getUpdater() { return type == BinderType::NEW ? this : nullptr; }
    virtual void match() { 
        result = domain.lookup(*repr, type);
        firstMatch = result;
    }
    virtual bool next() {
        bool ret = firstMatch;
        firstMatch = false;
        return ret;
    }
    virtual bool update() { return domain.check(*repr, imported); }
    virtual void print(std::ostream &out) const { out << *repr << "@" << type; }
    virtual ~PosMatcher() { };

    Match      &result;
    DomainType &domain;
    UTerm       repr;
    BinderType  type;
    unsigned    imported = 0;
    bool        firstMatch = false;
};

// }}}
// {{{ definition of make_binder

// Note: does not (quite) work this way
//       but it should be enough to make imported part of the key of full_indices

template <class Element>
inline UIdx make_binder(AbstractDomain<Element> &domain, NAF naf, Term const &repr, typename AbstractDomain<Element>::element_type *&elem, BinderType type, bool recursive, Term::VarSet &bound, int imported) {
    using DomainType          = AbstractDomain<Element>;
    using PredicateMatcher    = Matcher<Element>;
    using PosPredicateMatcher = PosMatcher<Element>;
    using PosPredicateBinder  = PosBinder<typename DomainType::bind_index_type&, SValVec>;
    using FullPredicateBinder = PosBinder<typename DomainType::full_index_type&>;
    if (naf == NAF::POS) {
        UTerm predClone(repr.clone());
        VarTermBoundVec occs;
        predClone->collect(occs, false);
        bool hasUnbound = false;
        for (auto &x : occs) {
            if ((x.first->bindRef = bound.find(x.first->name) == bound.end())) {
                hasUnbound = true;
            }
        }
        if (hasUnbound) {
            Term::VarSet occBoundSet;
            VarTermVec occBound;
            for (auto &x : occs) {
                if (x.first->bindRef)                               { x.first->bindRef = bound.emplace(x.first->name).second; } 
                else if (occBoundSet.emplace(x.first->name).second) { occBound.emplace_back(*x.first); }
            }
            Term::RenameMap rename;
            UTerm idxClone(predClone->renameVars(rename));
            SValVec predBound, idxBound;
            for (VarTerm &x : occBound) {
                auto it(rename.find(x.name));
                predBound.emplace_back(x.ref);
                idxBound.emplace_back(it->second.second);
            }
            Term::VarSet empty;
            idxClone->bind(empty);
            if (occBound.empty()) { 
                auto &idx(domain.add(std::move(idxClone), imported));
                return make_unique<FullPredicateBinder>(std::move(predClone), elem, idx, type);
            }
            else {
                assert(imported == 0);
                auto &idx(domain.add(std::move(idxBound), std::move(idxClone)));
                return make_unique<PosPredicateBinder>(std::move(predClone), elem, idx, type, std::move(predBound));
            }
        }
        else if (recursive) {
            assert(imported == 0);
            Term::VarSet empty;
            predClone->bind(empty);
            return make_unique<PosPredicateMatcher>(elem, domain, std::move(predClone), type);
        }
        else { 
            assert(imported == 0);
            return make_unique<PredicateMatcher>(elem, domain, repr, RECNAF::POS); 
        }
    }
    else { 
        assert(imported == 0);
        return make_unique<PredicateMatcher>(elem, domain, repr, recnaf(naf, recursive));
    }
}

// }}}

} } // namespace Ground Gringo

#endif // _GRINGO_GROUND_BINDERS_HH
