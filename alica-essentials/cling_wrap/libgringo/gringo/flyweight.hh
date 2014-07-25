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

#ifndef _GRINGO_FLYWEIGHT_HH
#define _GRINGO_FLYWEIGHT_HH

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <functional>
#include <utility>
#include <iterator>
#include <cassert>
#include <stdexcept>
#include <array>
#include <gringo/utility.hh>

#include <iostream>

namespace Gringo {

// {{{ declaration of Indexed

template <class T, class R = unsigned> 
struct Indexed {
public:
    typedef T value_type;
    typedef R index_type;
    template <class... Args>
    index_type emplace(Args&&... args);
    index_type insert(value_type &&value);
    value_type erase(index_type uid);
    value_type &operator[](index_type uid);
private:
    std::vector<value_type> values_;
    std::vector<index_type> free_;
};

// }}}
// {{{ declaration of FlyweightCoder

template <class T>
struct FlyweightCoder {
    typedef T value_type;
    typedef T & return_type;
    static bool encoded(unsigned uid);
    static bool encodeVal(value_type const &val, unsigned &uid);
    static unsigned encodeUid(unsigned uid);
    static return_type decodeVal(unsigned uid);
    static unsigned decodeUid(unsigned uid);
};

// }}}
// {{{ declaration of Flyweight

template <class T, class C = FlyweightCoder<T>>
class Flyweight {
    struct Hash;
    struct Equal;
public:
    typedef T value_type;
    typedef C coder;
    typedef typename std::unordered_set<unsigned, Hash, Equal>::const_iterator key_iterator;
    typedef typename coder::return_type return_type;

    Flyweight(value_type &&val);
    Flyweight(unsigned uid);
    template<typename... Args>
    Flyweight(Args... args);

    static unsigned uid(value_type &&val);
    unsigned uid() const;
    return_type operator*() const;

    bool operator==(Flyweight const &other) const;
    bool operator!=(Flyweight const &other) const;
    bool operator<(Flyweight const &other) const;
    bool operator>(Flyweight const &other) const;
    bool operator<=(Flyweight const &other) const;
    bool operator>=(Flyweight const &other) const;

    // garbage collection
    static key_iterator beginKey();
    static key_iterator endKey();
    static void erase(unsigned uid);
    static void clear();

private:
    typedef std::unordered_set<unsigned, Hash, Equal> key_set_type;
    typedef std::vector<value_type> value_vector_type;
    typedef std::vector<unsigned> free_vector_type;

    static key_set_type set_;
    static value_vector_type values_;
    static free_vector_type free_;

    unsigned uid_;
};

template <class T, class C>
struct Flyweight<T, C>::Hash {
    size_t operator()(unsigned uid) const;
};

template <class T, class C>
struct Flyweight<T, C>::Equal {
    bool operator()(unsigned uidA, unsigned uidB) const;
};

// }}}
// {{{ declaration of FlyweightVec

template <class T>
class FlyweightVec {
private:
    struct Hash;
    struct Equal;

public:
    typedef T value_type;
    typedef std::pair<unsigned, unsigned> key_type;
    typedef std::vector<value_type> value_vector;
    typedef typename value_vector::const_iterator const_iterator;
    typedef typename std::unordered_set<key_type, Hash, Equal>::const_iterator key_iterator;

    FlyweightVec(value_vector const &val);
    FlyweightVec(std::initializer_list<value_type> val);
    FlyweightVec(unsigned length, unsigned offset);
    value_type const &operator[](unsigned pos) const;
    value_type const &front() const;
    value_type const &back() const;
    const_iterator begin()const;
    const_iterator end() const;
    unsigned size() const;
    unsigned offset() const;
    bool operator==(FlyweightVec const &x) const;
    bool operator<(FlyweightVec const &x) const;

    // garbage collection
    static key_iterator beginKey();
    static key_iterator endKey();
    static void erase(unsigned length, unsigned offset);
    static void clear();

private:
    template <class C>
    unsigned init(C val);
    
    static unsigned const small = 32;
    typedef std::array<std::vector<unsigned>, small> free_small_type;
    typedef std::unordered_map<unsigned, std::vector<unsigned>> free_big_type;
    typedef std::unordered_set<key_type, Hash, Equal> key_set_type;
    typedef std::vector<value_type> value_vector_type;

    static free_small_type freeSmall_;
    static free_big_type freeBig_;
    static key_set_type set_;
    static value_vector_type values_;

    unsigned length_;
    unsigned offset_;
};

template <class T>
struct FlyweightVec<T>::Hash {
    size_t operator()(key_type key) const;
};

template <class T>
struct FlyweightVec<T>::Equal {
    bool operator()(key_type keyA, key_type keyB) const;
};

// }}}

// {{{ defintion of Indexed<T>

template <class T, class R>
template <class... Args>
typename Indexed<T, R>::index_type Indexed<T, R>::emplace(Args&&... args) {
    if (free_.empty()) {
        values_.emplace_back(std::forward<Args>(args)...);
        return index_type(values_.size() - 1);
    }
    else {
        index_type uid = free_.back();
        values_[uid] = value_type(std::forward<Args>(args)...);
        free_.pop_back();
        return uid;
    }
}

template <class T, class R>
typename Indexed<T, R>::index_type Indexed<T, R>::insert(value_type &&value) {
    if (free_.empty()) {
        values_.push_back(std::move(value));
        return index_type(values_.size() - 1);
    }
    else {
        index_type uid = free_.back();
        values_[uid] = std::move(value);
        free_.pop_back();
        return uid;
    }
}

template <class T, class R>
typename Indexed<T, R>::value_type Indexed<T, R>::erase(index_type uid) {
    value_type val(std::move(values_[uid]));
    if (uid + 1 == values_.size()) { values_.pop_back(); }
    else { free_.push_back(uid); }
    return val;
}

template <class T, class R>
typename Indexed<T, R>::value_type &Indexed<T, R>::operator[](typename Indexed<T, R>::index_type uid) {
    return values_[uid];
}

// }}}
// {{{ defintion of FlyweightCoder<T>

template <class T>
bool FlyweightCoder<T>::encoded(unsigned) {
    return false;
}

template <class T>
bool FlyweightCoder<T>::encodeVal(value_type const &, unsigned &) {
    return false;
}

template <class T>
unsigned FlyweightCoder<T>::encodeUid(unsigned uid) {
    return uid;
}

template <class T>
typename FlyweightCoder<T>::return_type FlyweightCoder<T>::decodeVal(unsigned) {
    throw std::logic_error("no decodeVal function");
}

template <class T>
unsigned FlyweightCoder<T>::decodeUid(unsigned uid) {
    return uid;
}

// }}}
// {{{ defintion of Flyweight<T, C>::Hash

template <class T, class C>
size_t Flyweight<T, C>::Hash::operator()(unsigned uid) const {
    return std::hash<value_type>()(values_[uid]);
}

// }}}
// {{{ defintion of Flyweight<T, C>::Equal

template <class T, class C>
bool Flyweight<T, C>::Equal::operator()(unsigned uidA, unsigned uidB) const {
    return values_[uidA] == values_[uidB];
}

// }}}
// {{{ defintion of Flyweight<T, C>

template <class T, class C>
Flyweight<T, C>::Flyweight(value_type &&val)
    : uid_(Flyweight::uid(std::move(val))) { }

template <class T, class C>
unsigned Flyweight<T, C>::uid(value_type &&val) {
    unsigned uid;
    if (!coder::encodeVal(val, uid)) {
        if (free_.empty()) {
            free_.push_back(values_.size());
            values_.emplace_back(std::move(val));
        }
        else {
            values_[free_.back()] = std::move(val);
        }
        auto res = set_.insert(free_.back());
        if (res.second) { free_.pop_back(); }
        return coder::encodeUid(*res.first);
    }
    else { return uid; }
}


template <class T, class C>
Flyweight<T, C>::Flyweight(unsigned uid)
    : uid_(uid) { }

template <class T, class C>
template<typename... Args>
Flyweight<T, C>::Flyweight(Args... args)
    : Flyweight(value_type(args...)) { }

template <class T, class C>
unsigned Flyweight<T, C>::uid() const {
    return uid_;
}

template <class T, class C>
bool Flyweight<T, C>::operator==(Flyweight const &other) const {
    return uid_ == other.uid_;
}

template <class T, class C>
bool Flyweight<T, C>::operator!=(Flyweight const &other) const {
    return uid_ != other.uid_;
}

template <class T, class C>
bool Flyweight<T, C>::operator<(Flyweight const &other) const {
    return **this < *other;
}

template <class T, class C>
bool Flyweight<T, C>::operator>(Flyweight const &other) const {
    return **this > *other;
}

template <class T, class C>
bool Flyweight<T, C>::operator<=(Flyweight const &other) const {
    return **this <= *other;
}

template <class T, class C>
bool Flyweight<T, C>::operator>=(Flyweight const &other) const {
    return **this >= *other;
}

template <class T, class C>
typename Flyweight<T, C>::return_type Flyweight<T, C>::operator*() const {
    if (coder::encoded(uid_)) { return coder::decodeVal(uid_); }
    else { return values_[coder::decodeUid(uid_)]; }
}

template <class T, class C>
typename Flyweight<T, C>::key_iterator Flyweight<T, C>::beginKey() {
    return set_.begin();
}

template <class T, class C>
typename Flyweight<T, C>::key_iterator Flyweight<T, C>::endKey() {
    return set_.end();
}

template <class T, class C>
void Flyweight<T, C>::erase(unsigned uid) {
    if (!C::encoded(uid)) {
        set_.erase(uid);
        free_.push_back(uid);
    }
}

template <class T, class C>
void Flyweight<T, C>::clear()
{
    set_    = key_set_type();
    free_   = free_vector_type();
    values_ = value_vector_type();
}

template <class T, class C>
typename Flyweight<T, C>::key_set_type Flyweight<T, C>::set_;

template <class T, class C>
typename Flyweight<T, C>::value_vector_type Flyweight<T, C>::values_;

template <class T, class C>
typename Flyweight<T, C>::free_vector_type Flyweight<T, C>::free_;

// }}}
// {{{ defintion of FlyweightVec<T>::Hash

template <class T>
size_t FlyweightVec<T>::Hash::operator()(key_type key) const {
    size_t seed = std::hash<unsigned>()(key.first);
    for (auto it = values_.begin() + key.second, end = it + key.first; it != end; ++it) { hash_combine(seed, *it); }
    return seed;
}

// }}}
// {{{ defintion of FlyweightVec<T>::Equal

template <class T>
bool FlyweightVec<T>::Equal::operator()(key_type keyA, key_type keyB) const
{
    if (keyA.first != keyB.first) { return false; }
    if (keyA.second == keyB.second) { return true; }
    auto a = values_.begin() + keyA.second;
    auto b = values_.begin() + keyB.second;
    return std::equal(a, a + keyA.first, b);
}

// }}}
// {{{ defintion of FlyweightVec<T>

template <class T>
FlyweightVec<T>::FlyweightVec(value_vector const &val)
    : length_(val.size())
    , offset_(init<value_vector const &>(val)) { }

template <class T>
FlyweightVec<T>::FlyweightVec(std::initializer_list<value_type> val)
    : length_(val.size())
    , offset_(init<std::initializer_list<value_type>>(val)) { }

template <class T>
template <class C>
unsigned FlyweightVec<T>::init(C val) {
    auto &free(length_ < small ? freeSmall_[length_] : freeBig_[length_]);
    if (free.empty()) {
        unsigned offset = values_.size();
        values_.insert(values_.end(), val.begin(), val.end());
        auto ret = set_.insert(key_type(length_, offset));
        if (!ret.second) {
            free.push_back(offset);
            return ret.first->second;
        }
        else { return  offset; }
    }
    else {
        unsigned offset = free.back();
        std::copy(val.begin(), val.end(), values_.begin() + offset);
        auto ret = set_.insert(key_type(length_, offset));
        if (ret.second) {
            free.pop_back();
            return offset;
        }
        else { return ret.first->second; }
    }
}

template <class T>
FlyweightVec<T>::FlyweightVec(unsigned length, unsigned offset)
    : length_(length)
    , offset_(offset) { }

template <class T>
typename FlyweightVec<T>::value_type const &FlyweightVec<T>::operator[](unsigned pos) const {
    assert(pos < length_);
    return values_[offset_ + pos];
}

template <class T>
typename FlyweightVec<T>::value_type const &FlyweightVec<T>::front() const { return *begin(); }
template <class T>
typename FlyweightVec<T>::value_type const &FlyweightVec<T>::back() const  { return *(end() - 1); }

template <class T>
typename FlyweightVec<T>::const_iterator FlyweightVec<T>::begin() const {
    assert(offset_ <= values_.size());
    return values_.begin() + offset_;
}

template <class T>
typename FlyweightVec<T>::const_iterator FlyweightVec<T>::end() const {
    assert(offset_ + length_ <= values_.size());
    return values_.begin() + offset_ + length_;
}

template <class T>
unsigned FlyweightVec<T>::size() const {
    return length_;
}

template <class T>
unsigned FlyweightVec<T>::offset() const {
    return offset_;
}

template <class T>
bool FlyweightVec<T>::operator==(FlyweightVec const &x) const {
    return offset_ == x.offset_ && length_ == x.length_;
}

template <class T>
bool FlyweightVec<T>::operator<(FlyweightVec const &x) const {
    if (size() != x.size()) { return size() < x.size(); }
    return std::lexicographical_compare(begin(), end(), x.begin(), x.end());
}

template <class T>
typename FlyweightVec<T>::key_iterator FlyweightVec<T>::beginKey() {
    return set_.begin();
}

template <class T>
typename FlyweightVec<T>::key_iterator FlyweightVec<T>::endKey() {
    return set_.end();
}

template <class T>
void FlyweightVec<T>::erase(unsigned length, unsigned offset) {
    set_.erase(key_type(length, offset));
    if (length < small) {
        freeSmall_[length].push_back(offset);
    }
    else {
        freeBig_[length].push_back(offset);
    }
}

template <class T>
void FlyweightVec<T>::clear() {
    freeSmall_ = free_small_type();
    freeBig_   = free_big_type();
    set_       = key_set_type();
    values_    = value_vector_type();
}

template <class T>
typename FlyweightVec<T>::free_small_type FlyweightVec<T>::freeSmall_;

template <class T>
typename FlyweightVec<T>::free_big_type FlyweightVec<T>::freeBig_;

template <class T>
typename FlyweightVec<T>::key_set_type FlyweightVec<T>::set_;

template <class T>
typename FlyweightVec<T>::value_vector_type FlyweightVec<T>::values_;

// }}}

} // namespace Gringo

namespace std {

// {{{ definition of hash functions

template <class T, class C>
struct hash<Gringo::Flyweight<T, C>> {
    size_t operator()(Gringo::Flyweight<T, C> const &f) const {
        return f.uid();
    }
};

template <class T>
struct hash<Gringo::FlyweightVec<T>> {
    size_t operator()(Gringo::FlyweightVec<T> const &f) const {
        size_t seed = typeid(Gringo::FlyweightVec<T>).hash_code();
        for (auto &x : f) { Gringo::hash_combine(seed, std::hash<T>()(x)); }
        return seed;
    }
};

// }}}

} // namespace std

#endif  // _GRINGO_FLYWEIGHT_HH
