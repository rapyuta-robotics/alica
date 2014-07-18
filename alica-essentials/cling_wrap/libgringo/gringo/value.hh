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

#ifndef _GRINGO_VALUE_HH
#define _GRINGO_VALUE_HH

#include <cstdint>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <string>
#include <functional>
#include <cassert>
#include <iterator>
#include <algorithm>
#include <utility>
#include <gringo/flyweight.hh>

namespace Gringo {

struct Value;
using ValVec    = std::vector<Value>;
using FWValVec  = FlyweightVec<Value>;
using FWString  = Flyweight<std::string>;
using IdValMap = std::unordered_map<FWString, Value>;

inline std::ostream &operator<<(std::ostream &out, FWString const &x) {
    out << *x;
    return out;
}

// {{{ declaration of quote/unquote

std::string quote(std::string const &str);
std::string unquote(std::string const &str);

// }}}
// {{{ declaration of Signature

struct Signature {
    struct Coder;

    Signature(FWString name, unsigned length);

    FWString name() const;
    unsigned length() const;

    template <bool enc>
    bool encode(unsigned &uid) const;

    size_t hash() const;
    bool operator==(Signature const &other) const;
    bool operator!=(Signature const &other) const;
    bool operator<(Signature const &other) const;

private:
    Flyweight<std::string> name_;
    unsigned length_;
};

inline std::ostream &operator<<(std::ostream &out, Signature const &x) {
    out << *x.name() << "/" << x.length();
    return out;
}

struct Signature::Coder {
    typedef Signature value_type;
    typedef Signature return_type;
    static bool encoded(unsigned uid);
    static bool encodeVal(Signature const &val, unsigned &uid);
    static unsigned encodeUid(unsigned uid);
    static Signature decodeVal(unsigned uid);
    static unsigned decodeUid(unsigned uid);
};

typedef Flyweight<Signature, Signature::Coder> FWSignature;

inline std::ostream &operator<<(std::ostream &out, FWSignature const &x) {
    out << *x;
    return out;
}

// }}}
// {{{ declaration of Value

struct Value {
    enum Type : unsigned { INF, NUM, ID, STRING, FUNC, SPECIAL, SUP };
    struct POD;

    // construction
    Value(const char *val, bool id = true);
    Value(std::string &&val, bool id = true);
    Value(FWString val, bool id = true);
    Value(int num);
    Value(bool inf);
    Value(ValVec const &val);
    Value(FWValVec val);
    Value(char const *name, ValVec const &val);
    Value(std::string &&name, ValVec const &val);
    Value(FWString name, FWValVec val);
    Value(Value a, Value b);
    Value();
    
    // value retrieval
    Type type() const;
    int num() const;
    FWString string() const;
    FWSignature sig() const;
    FWString name() const;
    FWValVec args() const;
    Value replace(IdValMap const &rep) const;

    // comparison
    size_t hash() const;
    bool operator==(Value const &other) const;
    bool less(Value const &other) const;
    bool operator!=(Value const &other) const;
    bool operator<(Value const &other) const;
    bool operator>(Value const &other) const;
    bool operator<=(Value const &other) const;
    bool operator>=(Value const &other) const;

    // ouput
    void print(std::ostream& out) const;

    operator POD&();
    operator POD const &() const;

    static unsigned const typeBits = 4u;
    static unsigned const typeMask = 1u+2u+4u+8u;

private:
    Value (unsigned type, unsigned value);

    unsigned type_;
    unsigned value_;
};

struct Value::POD {
    operator Value&();
    operator Value const &() const;
    Value *operator->();
    Value const *operator->() const;
    unsigned type;
    unsigned value;
};

std::ostream& operator<<(std::ostream& out, const Gringo::Value& val);

// }}}

// {{{ definition of Signature::Coder

inline bool Signature::Coder::encoded(unsigned uid) {
    return uid & 1u;
}

inline bool Signature::Coder::encodeVal(Signature const &val, unsigned &uid) {
    return val.encode<sizeof(unsigned) >= 4>(uid);
}

inline unsigned Signature::Coder::encodeUid(unsigned uid) {
    return uid << 1u;
}

inline Signature Signature::Coder::decodeVal(unsigned uid) {
    assert(uid & 1u);
    return Signature(uid >> 4u, uid >> 1u & (1u + 2u + 4u));
}

inline unsigned Signature::Coder::decodeUid(unsigned uid) {
    return uid >> 1u;
}

// }}}
// {{{ definition of Signature

inline Signature::Signature(FWString name, unsigned length)
    : name_(name)
    , length_(length) { }

inline FWString Signature::name() const {
    return name_;
}

inline unsigned Signature::length() const {
    return length_;
}

inline size_t Signature::hash() const {
    return get_value_hash(name_.uid(), length_);
}

inline bool Signature::operator==(Signature const &other) const {
    return name_ == other.name_ && length_ == other.length_;
}

inline bool Signature::operator!=(Signature const &other) const {
    return not operator==(other);
}

inline bool Signature::operator<(Signature const &other) const {
    if (length_ != other.length_) { return length_ < other.length_; }
    return name_ < other.name_;
}

template<bool enc>
bool Signature::encode(unsigned &uid) const {
    if (enc && length_ < 8 && name_.uid() < 16777216u) { 
        uid = (name_.uid() << 4u) | (length_ << 1u) | 1u;
        return true;
    }
    return false;
}

// }}}
// {{{ definition of Value::POD
    
inline Value::POD::operator Value&() {
    return *reinterpret_cast<Value*>(this);
}

inline Value::POD::operator Value const &() const {
    return *reinterpret_cast<Value const *>(this);
}

inline Value *Value::POD::operator->() {
    return reinterpret_cast<Value *>(this);
}

inline Value const *Value::POD::operator->() const {
    return reinterpret_cast<Value const *>(this);
}

// }}}
// {{{ definition of Value

inline Value::Value(unsigned type, unsigned value)
    : type_(type)
    , value_(value) { }

inline Value::Value() 
    : Value(SPECIAL, 0) { }

inline Value::Value(bool inf)
    :  Value(inf ? INF : SUP, 0) { }

inline Value::Value(int val)
    : Value(NUM, static_cast<unsigned>(val)) { }

inline Value::Value(const char *val, bool id)
    : Value(std::string(val), id) { }

inline Value::Value(std::string &&val, bool id)
    : Value(FWString(std::move(val)), id) { }

inline Value::Value(FWString val, bool id)
    : Value(id ? ID : STRING, val.uid()) { }

inline Value::Value(ValVec const &args)
    : Value(FWValVec(args)) { }

inline Value::Value(FWValVec args)
    : Value("", args) { }

inline Value::Value(char const *name, ValVec const &val)
    : Value(std::string(name), val) { }

inline Value::Value(std::string &&name, ValVec const &val)
    : Value(FWString(name), FWValVec(val)) { }

inline Value::Value(FWString name, FWValVec args)
    : Value(FUNC | (FWSignature(name, args.size()).uid() << typeBits),  args.offset()) {
}

inline void Value::print(std::ostream& out) const {
    switch(type()) {
        case NUM:    { out << num(); break; }
        case ID:     { out << *string(); break; }
        case STRING: { out << '"' << quote(*string()) << '"'; break; }
        case INF:    { out << "#inf"; break; }
        case SUP:    { out << "#sup"; break; }
        case FUNC:   {
            out << *name();
            auto a = args();
            out << "(";
            if (a.size() > 0) {
                std::copy(a.begin(), a.end() - 1, std::ostream_iterator<Value>(out, ","));
                out << *(a.end() - 1);
            }
            out << ")";
            break;
        }
        case SPECIAL: { out << "#special"; break; }
    }
}

inline int ipow(int a, int b) {
    if (b < 0) { return 0; }
    else {
        int r = 1;
        while (b > 0) {
            if(b & 1) { r *= a; }
            b >>= 1;
            a *= a;
        }
        return r;
    }
}

inline size_t Value::hash() const {
    return get_value_hash(type_, value_);
}

inline bool Value::operator==(Value const &other) const {
    return type_ == other.type_ && value_ == other.value_;
}

inline bool Value::less(Value const &other) const {
    if (type() != other.type()) { return type() < other.type(); }
    switch(type()) {
        case NUM:       { return num() < other.num(); }
        case ID:
        case STRING:    { return *string() < *other.string(); }
        case INF:       { return false; }
        case SUP:       { return false; }
        case FUNC: {
            auto aa = args(), ab = other.args();
            if (aa.size() != ab.size()) { return aa.size() < ab.size(); }
            auto na = name(), nb = other.name();
            if (na != nb) { return na < nb; }
            return std::lexicographical_compare(aa.begin(), aa.end(), ab.begin(), ab.end());
        }
        case SPECIAL: { return false; }
    }
    assert(false);
    return false;
}

inline Value Value::replace(IdValMap const &rep) const {
    switch(type()) {
        case ID:        {
            auto it = rep.find(name());
            if (it != rep.end()) { return it->second; }
        }
        case NUM:
        case INF:
        case SUP:
        case SPECIAL:
        case STRING:    { return *this; }
        case FUNC: {
            ValVec vals;
            for (auto &x : args()) { vals.emplace_back(x.replace(rep)); }
            return Value(name(), vals);
        }
    }

}

inline bool Value::operator!=(Value const &other) const {
    return !(*this == other);
}

inline bool Value::operator<(Value const &other) const {
    return (*this != other) && this->less(other); 
}

inline bool Value::operator>(Value const &other) const {
    return (*this != other) && other.less(*this);
}

inline bool Value::operator<=(Value const &other) const {
    return (*this == other) || this->less(other);
}

inline bool Value::operator>=(Value const &other) const {
    return (*this == other) || other.less(*this);
}

inline Value::Type Value::type() const {
    return Type(type_ & unsigned(typeMask));
}

inline int Value::num() const {
    assert(type() == NUM);
    return static_cast<int>(value_);
}

inline FWString Value::string() const {
    assert(type() == STRING || type() == ID);
    return value_;
}

inline FWSignature Value::sig() const {
    switch (type()) {
        case FUNC: { return FWSignature(type_ >> typeBits); }
        case ID:   { return FWSignature(string(), 0); }
        default:   { assert(false); }
    }
    return FWSignature("", 0);
}

inline FWString Value::name() const {
    switch (type())  {
        case FUNC: { return (*FWSignature(type_ >> typeBits)).name(); }
        case ID:   { return value_; }
        default:   { assert(false); } 
    }
    return 0u;
}

inline FWValVec Value::args() const {
    assert(type() == FUNC);
    return FWValVec((*FWSignature(type_ >> typeBits)).length(), value_);
}

inline Value::operator Value::POD&() {
    return *reinterpret_cast<POD*>(this);
}

inline Value::operator Value::POD const &() const {
    return *reinterpret_cast<POD const *>(this);
}

inline std::ostream& operator<<(std::ostream& out, Gringo::Value const &val) {
    val.print(out);
    return out;
}

// }}}
// {{{ definition of quote/unquote

inline std::string quote(const std::string &str) {
    std::string res;
    for (char c : str) {
        switch (c) {
            case '\n': {
                res.push_back('\\');
                res.push_back('n');
                break;
            }
            case '\\': {
                res.push_back('\\');
                res.push_back('\\');
                break;
            }
            case '"': {
                res.push_back('\\');
                res.push_back('"');
                break;
            }
            default: {
                res.push_back(c);
                break;
            }
        }
    }
    return res;
}

inline std::string unquote(const std::string &str) {
    std::string res;
    bool slash = false;
    for (char c : str) {
        if (slash) {
            switch (c) {
                case 'n': {
                    res.push_back('\n');
                    break;
                }
                case '\\': {
                    res.push_back('\\');
                    break;
                }
                case '"': {
                    res.push_back('"');
                    break;
                }
                default: {
                    assert(false);
                    break;
                }
            }
            slash = false;
        }
        else if (c == '\\') { slash = true; }
        else { res.push_back(c); }
    }
    return res;
}

// }}}

} // namespace Gringo

namespace std {

// {{{ definition of hash functions for Signature and Value

template<>
struct hash<Gringo::Signature> {
    size_t operator()(Gringo::Signature const &sig) const { return sig.hash(); }
};

template<>
struct hash<Gringo::Value> {
    size_t operator()(Gringo::Value const &val) const { return val.hash(); }
};

// }}}

} // namespace std

#endif // _GRINGO_VALUE_HH

