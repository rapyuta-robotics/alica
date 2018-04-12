#pragma once
#include <stdint.h>
#include <assert.h>
#include "engine/blackboard/BBIdent.h"
namespace alica {

class Variant {
    
    enum Type {
        TypeDouble=0, //mean optimization for the gradient solver case
        TypeFloat, 
        TypeBool,
        TypePtr,
        TypeInt,
        TypeIdent,
        TypeNone
    }
    public:
        constexpr int kUnionSize = std::max({sizeof(double), sizeof(void*), sizeof(BBIdent)});
        constexpr int kVariantSize = kUnionSize +1;

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

        explicit Variant(BBIdent id)
            : _value.asIdent(id)
            , _type(TypeIdent) {}
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
        BBIdent getIdent() const {assert(_type == TypeIDent); return _value.asIdent;}

        //Set:
        void setDouble(double d) {_type=TypeDouble; _value.asDouble = d;}
        void setFloat(float f) {_type=TypeFloat; _value.asFloat = f;}
        void setInt(int64_t i) {_type=TypeInt; _value.asInt = i;}
        void setBool(bool b) {_type=TypeBool; _value.asBool = b;}
        void setPtr(void* ptr) {_type=TypePtr; _value.asPtr = ptr;}
        void setIDent(BBIdent id) {_type=TypeIdent; _value.asIdent = id;}


        void serializeTo(uint8_t& arr[kVariantSize]) const {
            arr[0] = _type;
            memcpy(arr+1, &_value.asRaw, kUnionSize);
        }
        void loadFrom(const uint8_t& arr[kVariantSize]) {
            _type = arr[0];
            memcpy(&value.asRaw, arr+1, kUnionSize);
        }


    private:
    union Data {
        double asDouble;
        float asFloat;
        bool asBool;
        void* asPtr;
        int64_t asInt;
        BBIdent asIdent;
        uint8_t asRaw[kVariantSize]
    } _value;
    STATIC_ASSERT(sizeof(Data==kUnionSize), "Unexpected union size!");
    Type _type;
};



}