#pragma once
#include <stdint.h>
#include <assert.h>

namespace alica {

class Variant {
    enum Type {
        TypeDouble=0, //mean optimization for the gradient solver case
        TypeFloat, 
        TypeBool,
        TypePtr,
        TypeInt,
        TypeNone
    }
    public:
        Variant()
            : _type(TypeNone) {}

        explicit Variant(double d)
            : _value.asDouble(d)
            , _type(TypeDouble) {}

        explicit Variant(float f)
            : _value.asFloat(f)
            , _type(TypeFloat) {}

        explicit Variant(int64_t i)
            : _value.asInt(i)
            , _type(TypeInt) {}

        explicit Variant(bool b)
            : _value.asBool(b)
            , _type(TypeBool) {}

        explicit Variant(void* ptr)
            : _value.asPtr(ptr)
            , _type(TypePtr) {}
        //Test:
        bool isSet() const {return _type != TypeNone;}
        bool isDouble() const {return _type == TypeDouble;}
        bool isFloat() const {return _type == TypeFloat;}
        bool isInt() const {return _type == TypeInt;}
        bool isBool() const {return _type == TypeBool;}
        bool isPtr() const {return _type == TypePtr;}
        
        //Get:
        
        double getDouble() const {assert(_type == TypeDouble); return _value.asDouble;}
        float getFloat() const {assert(_type == TypeFloat); return _value.asFloat;}
        int getInt() const {assert(_type == TypeInt); return _value.asInt;}
        bool getBool() const {assert(_type == TypeBool); return _value.asBool;}
        void* getPtr() const {assert(_type == TypePtr); return _value.asPtr;}

        //Set:
        void setDouble(double d) {_type=TypeDouble; _value.asDouble = d;}
        void setFloat(float f) {_type=TypeFloat; _value.asFloat = f;}
        void setInt(int64_t i) {_type=TypeInt; _value.asInt = i;}
        void setBool(bool b) {_type=TypeBool; _value.asBool = b;}
        void setPtr(void* ptr) {_type=TypePtr; _value.asPtr = ptr;}


    private:
    union {
        double asDouble;
        float asFloat;
        bool asBool;
        void* asPtr;
        int64_t asInt;
    } _value;
    _type;
};



}