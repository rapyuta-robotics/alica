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

#ifndef _GRINGO_INTERVALS_HH
#define _GRINGO_INTERVALS_HH

#include <vector>
#include <iostream>
#include <algorithm>

namespace Gringo {

// NOTE: refactor this is almost the same as interval_set ...

template <class T>
struct IntervalSet {
    // Notes: read < as before with gap (except if bounds of same type are compared)
    using Value = T;
    struct RBound;
    struct LBound {
        LBound &operator=(RBound const &x) {
            bound     = x.bound;
            inclusive = !x.inclusive;
            return *this;
        }
        Value bound;
        bool inclusive;
        bool operator<(LBound const &x) const  { return bound < x.bound || (!(x.bound < bound) &&  inclusive && !x.inclusive ); }
        bool operator<=(LBound const &x) const { return bound < x.bound || (!(x.bound < bound) && (inclusive || !x.inclusive)); }
        bool operator<(RBound const &x) const  { return bound < x.bound || (!(x.bound < bound) &&  inclusive &&  x.inclusive ); }
    };
    struct RBound {
        RBound &operator=(LBound const &x) {
            bound     = x.bound;
            inclusive = !x.inclusive;
            return *this;
        }
        Value bound;
        bool inclusive;
        bool operator<(RBound const &x) const  { return bound < x.bound || (!(x.bound < bound) &&  !inclusive &&  x.inclusive ); }
        bool operator<=(RBound const &x) const { return bound < x.bound || (!(x.bound < bound) && (!inclusive ||  x.inclusive)); }
        bool operator<(LBound const &x) const  { return bound < x.bound || (!(x.bound < bound) &&  !inclusive && !x.inclusive ); }
    };
    struct Interval {
        bool contains(T const &x) const         { return !(x < *this) && !(*this < x); }
        bool empty() const                      { return !(left < right); }
        bool operator<(Interval const &x) const { return right < x.left; }
        LBound left;
        RBound right;
    };
    using IntervalVec = std::vector<Interval>;
    using const_iterator = typename IntervalVec::const_iterator;

    void add(Interval const &x) {
        if (!x.empty()) {
            auto it = std::lower_bound(vec.begin(), vec.end(), x);
            if (it == vec.end()) { vec.emplace_back(x); }
            else {
                auto jt = std::upper_bound(it, vec.end(), x);
                if (it == jt) { vec.emplace(it, x); }
                else {
                    it->left  = std::min(x.left, it->left);
                    it->right = std::max(x.right, (jt-1)->right);
                    vec.erase(it+1, jt);
                }
            }
        }
    }
    void remove(Interval const &x) {
        if (!x.empty()) {
            auto it = std::lower_bound(vec.begin(), vec.end(), x);
            if (it != vec.end()) {
                auto jt = std::upper_bound(it, vec.end(), x);
                if (it+1 == jt) {
                    Interval r;
                    r.left    = x.right;
                    r.right   = it->right;
                    it->right = x.left;
                    if (it->empty()) {
                        if (r.empty()) { vec.erase(it); }
                        else           { *it = r; }
                    }
                    else if (!r.empty()) { vec.emplace(it+1, r); }
                }
                else if (it != jt) {
                    it->right    = x.left;
                    (jt-1)->left = x.right;
                    vec.erase(it + !it->empty(), jt - !(jt-1)->empty());
                }
            }
        }
    }
    bool contains(Interval const &x) const { 
        if (x.empty()) { return true; }
        for (auto &y : vec) {
            if (x.right <= y.right) { return y.left <= x.left; }
        }
        return false;
    }
    bool intersects(Interval const &x) const { 
        if (!x.empty()) {
            for (auto &y : vec) {
                if (x.left < y.right) { return y.left < x.right; }
            }
        }
        return false;
    }
    bool empty() const { return vec.empty(); }
    void clear() { return vec.clear(); }
    void add(Value const &a, bool ta, Value const &b, bool tb) { return add({{a, ta}, {b, tb}}); }
    const_iterator begin() const { return vec.begin(); }
    const_iterator end() const { return vec.end(); }
    void remove(Value const &a, bool ta, Value const &b, bool tb) { return remove({{a, ta}, {b, tb}}); }

    IntervalVec vec;
};

template <class T>
bool operator<(T const &a, typename IntervalSet<T>::Interval const &b) {
    return a < b.left.bound || (!(b.left.bound < a) && !b.left.inclusive);
}

template <class T>
bool operator<(typename IntervalSet<T>::Interval const &a, T const &b) {
    return a.right.bound < b || (!(b < a.right.bound) && !a.right.inclusive);
}

template <class T>
class enum_interval_set {
public:
    using value_type = T;

private:
    struct Interval {
        Interval(value_type left, value_type right) 
            : left(left)
            , right(right) { }
        bool contains(value_type const &x) const { return !(x < left) && x < right; }
        bool empty() const                       { return !(left < right); }
        bool operator<(Interval const &x) const  { return right < x.left; }
        T left;
        T right;
    };
    using interval_vec = std::vector<Interval>;

public:
    class const_iterator : public std::iterator<std::bidirectional_iterator_tag, value_type const, int> { 
        friend class enum_interval_set;
        using iterator = std::iterator<std::bidirectional_iterator_tag, value_type const, int>;
    public:
        const_iterator() = default;
        const_iterator(const_iterator const &x) = default;

        const_iterator& operator=(const const_iterator&) = default;
        bool operator==(const_iterator const &x) const { return current == x.current; }
        bool operator!=(const_iterator const &x) const { return current != x.current; }

        const_iterator& operator++() {
            ++current;
            if (!(current < rng->right)) {
                ++rng;
                if (rng != set->end()) { current = rng->left; }
            }
            return *this;
        }
        const_iterator operator++(int) {
            const_iterator x = *this;
            ++(*this);
            return x;
        }
        const_iterator& operator--() {
            --current;
            if (rng == set->end()) { --rng; }
            if (current < rng->left) {
                if (rng != set->begin()) { 
                    --rng;
                    current = rng->right; 
                    --current;
                }
            }
            return *this;
        }
        const_iterator operator--(int) {
            const_iterator x = *this;
            --(*this);
            return x;
        }

        value_type operator*() const { return current; }
        typename iterator::pointer operator->() const  { return &current; }

    private:
        const_iterator(value_type const &current, typename interval_vec::const_iterator const &rng, interval_vec const &set)
            : current(current)
            , rng(rng)
            , set(&set) { }
        value_type current;
        typename interval_vec::const_iterator rng;
        interval_vec const                   *set;

    };
    
    enum_interval_set()                           = default;
    enum_interval_set(enum_interval_set const &x) = default;
    enum_interval_set(enum_interval_set &&x)      = default;
    void add(value_type const &a, value_type const &b)              { return add(Interval(a, b)); }
    void remove(value_type const &a, value_type const &b)           { return remove(Interval(a, b)); }
    void intersect(enum_interval_set const &set) {
        auto it = vec.begin();
        interval_vec intersection;
        for (auto &x : set.vec) {
            // Note: that could be a call to lower bound too ...
            for (; it != vec.end() && it->right <= x.left; ++it) { }
            for (; it != vec.end() && it->right <= x.right; ++it) { 
                intersection.emplace_back(std::max(it->left, x.left), it->right);
            }
            if (it != vec.end() && it->left < x.right) {
                intersection.emplace_back(std::max(it->left, x.left), x.right);
            }
        }
        std::swap(intersection, vec);
    }
    bool contains(value_type const &a, value_type const &b) const   { return contains(Interval(a, b)); }
    bool intersects(value_type const &a, value_type const &b) const { return intersects(Interval(a, b)); }
    bool empty() const            { return vec.empty(); }
    void clear()                  { return vec.clear(); }
    value_type front() const      { return vec.front().left; }
    value_type back() const       { auto ret = vec.back().right; --ret; return ret; }
    const_iterator begin() const  { return const_iterator(empty() ? value_type() : vec.front().left, vec.begin(), vec); }
    const_iterator end() const    { return const_iterator(empty() ? value_type() : vec.back().right, vec.end(), vec); }

private:
    void add(Interval const &x) {
        if (!x.empty()) {
            auto it = std::lower_bound(vec.begin(), vec.end(), x);
            if (it == vec.end()) { vec.emplace_back(x); }
            else {
                auto jt = std::upper_bound(it, vec.end(), x);
                if (it == jt) { vec.emplace(it, x); }
                else {
                    it->left  = std::min(x.left, it->left);
                    it->right = std::max(x.right, (jt-1)->right);
                    vec.erase(it+1, jt);
                }
            }
        }
    }
    void remove(Interval const &x) {
        if (!x.empty()) {
            auto it = std::lower_bound(vec.begin(), vec.end(), x);
            if (it != vec.end()) {
                auto jt = std::upper_bound(it, vec.end(), x);
                if (it+1 == jt) {
                    Interval r(x.right, it->right);
                    it->right = x.left;
                    if (it->empty()) {
                        if (r.empty()) { vec.erase(it); }
                        else           { *it = r; }
                    }
                    else if (!r.empty()) { vec.emplace(it+1, r); }
                }
                else if (it != jt) {
                    it->right = x.left;
                    (jt-1)->left = x.right;
                    vec.erase(it + !it->empty(), jt - !(jt-1)->empty());
                }
            }
        }
    }
    bool contains(Interval const &x) const { 
        if (x.empty()) { return true; }
        for (auto &y : vec) {
            if (!(y.right < x.right)) { return !(x.left < y.left); }
        }
        return false;
    }
    bool intersects(Interval const &x) const { 
        if (!x.empty()) {
            for (auto &y : vec) {
                if (x.left < y.right) { return y.left < x.right; }
            }
        }
        return false;
    }
    
    std::vector<Interval> vec;
};

} // namespace Gringo

namespace std {

template <class T>
typename Gringo::IntervalSet<T>::const_iterator begin(Gringo::IntervalSet<T> const &x) { return x.begin(); }

template <class T>
typename Gringo::IntervalSet<T>::const_iterator end(Gringo::IntervalSet<T> const &x) { return x.end(); }

} // namespace std

#endif // _GRINGO_INTERVALS_HH
