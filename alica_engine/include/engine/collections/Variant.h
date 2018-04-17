#pragma once
#include "engine/blackboard/BBIdent.h"
#include <stdint.h>
#include <assert.h>
#include <string.h> //for memcopy

namespace alica {

class Variant {
    
    enum Type {
        TypeDouble=0, //blatant optimization for the gradient solver case
        TypeFloat, 
        TypeBool,
        TypePtr,
        TypeInt,
        TypeIdent,
        TypeNone
    };
public:
    //annoyingly, C++11 does not yet allow std::max in constexpr (nor does it have an initializer list version)
    static constexpr size_t kUnionSize = sizeof(double) > sizeof(void*) ? sizeof(double) : sizeof(void*);
    static constexpr size_t kVariantSize = kUnionSize +1;

    Variant()
        : _type(TypeNone) {}

    explicit Variant(double d)
        : _value{d}
        ,  _type(TypeDouble) {}

    explicit Variant(float f)
        : _type(TypeFloat) { _value.asFloat = f; }

    explicit Variant(int64_t i)
        : _type(TypeInt) { _value.asInt = i;}

    explicit Variant(bool b)
        : _type(TypeBool) { _value.asBool = b;}

    explicit Variant(void* ptr)
        : _type(TypePtr) { _value.asPtr = ptr;}

    explicit Variant(BBIdent id)
        : _type(TypeIdent) {_value.asIdent = id;}

    //Test:
    bool isSet() const {return _type != TypeNone;}
    bool isDouble() const {return _type == TypeDouble;}
    bool isFloat() const {return _type == TypeFloat;}
    bool isInt() const {return _type == TypeInt;}
    bool isBool() const {return _type == TypeBool;}
    bool isPtr() const {return _type == TypePtr;}
    bool isIdent() const {return _type == TypeIdent;}

    //Get:

    double getDouble() const {assert(_type == TypeDouble); return _value.asDouble;}
    float getFloat() const {assert(_type == TypeFloat); return _value.asFloat;}
    int getInt() const {assert(_type == TypeInt); return _value.asInt;}
    bool getBool() const {assert(_type == TypeBool); return _value.asBool;}
    void* getPtr() const {assert(_type == TypePtr); return _value.asPtr;}
    BBIdent getIdent() const {assert(_type == TypeIdent); return _value.asIdent;}

    //Set:
    void setDouble(double d) {_type=TypeDouble; _value.asDouble = d;}
    void setFloat(float f) {_type=TypeFloat; _value.asFloat = f;}
    void setInt(int64_t i) {_type=TypeInt; _value.asInt = i;}
    void setBool(bool b) {_type=TypeBool; _value.asBool = b;}
    void setPtr(void* ptr) {_type=TypePtr; _value.asPtr = ptr;}
    void setIDent(BBIdent id) {_type=TypeIdent; _value.asIdent = id;}


    int serializeTo(uint8_t* arr) const {
        arr[0] = static_cast<uint8_t>(_type);
        memcpy(arr+1, &_value.asRaw, kUnionSize);
        return static_cast<int>(kVariantSize);
    }
    int loadFrom(const uint8_t* arr) {
        _type = static_cast<Type>(arr[0]);
        memcpy(&_value.asRaw, arr+1, kUnionSize);
        return static_cast<int>(kVariantSize);
    }


private:
    union Data {
        double asDouble;
        float asFloat;
        bool asBool;
        void* asPtr;
        int64_t asInt;
        BBIdent asIdent;
        uint8_t asRaw[kUnionSize];
    } _value; 
    static_assert(sizeof(Data)==kUnionSize, "Unexpected union size!");
    Type _type;
};



}