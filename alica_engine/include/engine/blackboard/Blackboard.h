#pragma once

#include "BlackboardBlueprint.h"
#include "engine/Types.h"
#include "engine/modelmanagement/Strings.h"
#include <any>
#include <array>
#include <cassert>
#include <exception>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <yaml-cpp/yaml.h>

namespace alica
{

class BlackboardUtil;

struct BlackboardException : public std::logic_error
{
    static constexpr const char* BB_EXCEPTION_PREFIX = "Blackboard exception: ";
    explicit BlackboardException(const std::string& errMsg)
            : std::logic_error(BB_EXCEPTION_PREFIX + errMsg)
    {
    }
    BlackboardException(const BlackboardException&) = default;
    BlackboardException& operator=(const BlackboardException&) = default;
};

class BlackboardImpl
{
    using BBValueType = std::variant<std::monostate, bool, int64_t, uint64_t, double, std::string, std::any>;
    static constexpr std::size_t BB_VALUE_TYPE_ANY_INDEX = 6;

    template <class... Args>
    static std::string stringify(Args&&... args)
    {
        std::ostringstream oss;
        (oss << ... << std::forward<Args>(args));
        return oss.str();
    }

public:
    BlackboardImpl() = default;
    BlackboardImpl(const BlackboardImpl&) = delete;
    BlackboardImpl(const BlackboardBlueprint* blueprint)
    {
        for (const auto& [key, keyInfo] : *blueprint) {
            auto typeIndex = getTypeIndex(keyInfo.type);
            if (!typeIndex.has_value() && keyInfo.type != "std::any") {
                throw BlackboardException(stringify("key: ", key, " has an unsupported type"));
            }
            _yamlType.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(keyInfo.type));
            if (keyInfo.defaultValue.has_value()) {
                if (keyInfo.type == "std::any") {
                    throw BlackboardException(stringify("key: ", key, " cannot have a default value because it is of type std::any"));
                }
                try {
                    _vals.emplace(std::piecewise_construct, std::forward_as_tuple(key),
                            std::forward_as_tuple(makeBBValueForIndex<true>::make(typeIndex.value(), keyInfo.defaultValue.value())));
                } catch (const std::exception& ex) {
                    throw BlackboardException(stringify("Could not set key: ", key, ", from default value: ", keyInfo.defaultValue.value(),
                            ", type: ", keyInfo.type, ", details: ", ex.what()));
                }
            } else {
                if (keyInfo.type == "std::any") {
                    // explicitly set the index for std::any, since getTypeIndex() returns empty for std::any
                    typeIndex = BB_VALUE_TYPE_ANY_INDEX;
                }
                try {
                    // insert a default constructed value
                    _vals.emplace(
                            std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(makeBBValueForIndex<false>::make(typeIndex.value())));
                } catch (const std::exception& ex) {
                    throw BlackboardException(stringify("Could not initialize key: ", key, " with value of type: ", keyInfo.type, ", details: ", ex.what()));
                }
            }
        }
    }

    template <typename T>
    const T& get(const std::string& key) const
    {
        // T must be an exact match to the type stored in the variant (i.e. no type conversions taken into account)
        // Exact matches are required because we return by reference
        try {
            const auto yamlTypeIt = _yamlType.find(key);
            if (yamlTypeIt == _yamlType.end()) {
                // key is not found in yaml file
                if constexpr (isTypeInVariant<T, BBValueType>::value) {
                    // T is a known type or std::any, in either case use std::get directly to fetch from variant
                    // Although std::any is considered an unknown type, we treat it as if its a known type here to avoid any_cast<const std::any&>
                    return std::get<T>(_vals.at(key));
                } else {
                    // T is an unknown type, use std::any for variant type & any_cast to T
                    return std::any_cast<const T&>(std::get<std::any>(_vals.at(key)));
                }
            } else {
                // key is found in yaml file
                if (yamlTypeIt->second == "std::any") {
                    // since yaml type is std::any, value would be stored as std::any, regardless of T being a known or unknown type
                    if constexpr (std::is_same_v<std::decay_t<T>, std::any>) {
                        // avoid any_cast<const std::any&>
                        return std::get<T>(_vals.at(key));
                    } else {
                        return std::any_cast<const T&>(std::get<std::any>(_vals.at(key)));
                    }
                }
                if constexpr (isTypeInVariant<T, BBValueType>::value) {
                    // T is a known type or std::any, in either case use std::get directly to fetch from variant
                    // if T is std::any, get<T> will throw, because yaml type is not "std::any"
                    return std::get<T>(_vals.at(key));
                } else {
                    // T is an unknown type, use std::any for variant type & any_cast to T
                    return std::any_cast<const T&>(std::get<std::any>(_vals.at(key)));
                }
            }
        } catch (const std::bad_variant_access& e) {
            // T is a known type or std::any, but type mismatch
            auto setType = getTypeName(_vals.at(key).index());
            auto getType = getTypeName<T>();
            setType = (setType == nullptr ? "std::any" : setType);
            getType = (getType == nullptr ? "std::any" : getType);
            throw BlackboardException(stringify("get() type mismatch, key: ", key, ", setType: ", setType, ", getType: ", getType));
        } catch (const std::bad_any_cast& e) {
            // T is an unknown type or yaml type is std::any, but type mismatch
            auto getType = getTypeName<T>();
            getType = (getType == nullptr ? "unknown" : getType);
            throw BlackboardException(stringify("get() type mismatch, key: ", key, ", setType: unknown, getType: ", getType));
        } catch (const std::out_of_range& e) {
            // key not set
            throw BlackboardException(stringify("get() failure, key: ", key, " is not yet set, so cannot get it"));
        }
    }

    template <typename T>
    T& get(const std::string& key)
    {
        return const_cast<T&>(static_cast<const BlackboardImpl*>(this)->get<T>(key));
    }

    template <class T>
    void set(const std::string& key, T&& value)
    {
        using Type = std::decay_t<T>;
        // Promote signed integers to int64 & unsigned integers to uint64 to avoid compiler error in variant constructor/assignment operator (due to ambiguity)
        using PromotedType = std::conditional_t<std::is_integral_v<Type> && !std::is_same_v<Type, bool>,
                std::conditional_t<std::is_signed_v<Type>, int64_t, uint64_t>, Type>;
        if constexpr (std::is_same_v<Type, PromotedType>) {
            setImpl(key, std::forward<T>(value));
        } else {
            setImpl(key, PromotedType(value));
        }
    }

    void map(const std::string& srcKey, const std::string& targetKey, const BlackboardImpl& srcBb)
    {
        // Note: srcKey & targetKey has to be set & both have to be found in the yaml (pml or beh) file
        std::visit(
                [srcKey, targetKey, this](auto&& srcValue) {
                    // set the target as a side effect
                    set(targetKey, std::forward<decltype(srcValue)>(srcValue));
                },
                srcBb._vals.at(srcKey));
    }

    bool hasValue(const std::string& key) const { return _vals.count(key); }
    void removeValue(const std::string& key) { _vals.erase(key); }

    void clear() { _vals.clear(); }
    bool empty() const { return _vals.empty(); }
    size_t size() const { return _vals.size(); }

private:
    std::unordered_map<std::string, BBValueType> _vals;
    std::unordered_map<std::string, std::string> _yamlType;

    friend BlackboardUtil;

    template <class T>
    void setImpl(const std::string& key, T&& value)
    {
        const char* type = getTypeName<T>();
        const auto yamlTypeIt = _yamlType.find(key);
        if (yamlTypeIt == _yamlType.end()) {
            // key is not found in yaml file
            if (!type) {
                // T is an unknown type, use std::any
                _vals[key] = std::any{std::forward<T>(value)};
            } else {
                // T is a known type, just assign
                _vals[key] = std::forward<T>(value);
            }
        } else {
            // key is found in yaml file
            const auto& yamlType = yamlTypeIt->second;
            if (yamlType == "std::any") {
                // yaml type is std::any, so can store values of any type, therefore use std::any for variant type
                _vals[key] = std::any{std::forward<T>(value)};
            } else {
                if (!type) {
                    // T is an unknown type, but yaml type is always known
                    throw BlackboardException(stringify("set() failure, unknown type used to set key: ", key, ", expected type (from yaml file): ", yamlType));
                } else if (yamlType != type) {
                    // T is a known type, but type mismatch
                    throw BlackboardException(
                            stringify("set() type mismatch, key: ", key, ", set type: ", type, ", expected type (from yaml file): ", yamlType));
                } else {
                    // T is a known type & types match, assign
                    _vals[key] = std::forward<T>(value);
                }
            }
        }
    }

    template <class T>
    static const char* getTypeName()
    {
        return getTypeName(BBValueType{T{}}.index());
    }

    static const char* getTypeName(std::size_t index)
    {
        if (index - 1 < BB_VALUE_TYPE_NAMES_SIZE) {
            // index - 1 to account for std::monostate in the variant
            return BB_VALUE_TYPE_NAMES[index - 1];
        }
        // Note: if index points to std::any, nullptr will be returned, because std::any is also considered to be an unknown type
        return nullptr;
    }

    static std::optional<std::size_t> getTypeIndex(const std::string& type)
    {
        for (std::size_t i = 0; i < BB_VALUE_TYPE_NAMES_SIZE; ++i) {
            if (BB_VALUE_TYPE_NAMES[i] == type) {
                // i + 1 to account for std::monostate in the variant
                return i + 1;
            }
        }
        return {};
    }

    static constexpr const char* BB_VALUE_TYPE_NAMES[] = {"bool", "int64", "uint64", "double", "std::string"};
    static constexpr std::size_t BB_VALUE_TYPE_NAMES_SIZE = sizeof(BB_VALUE_TYPE_NAMES) / sizeof(const char*);

    template <bool PARSE_ARGS = false>
    struct makeBBValueForIndex
    {
        template <class... Args>
        static BBValueType make(std::size_t index, Args&&... args)
        {
            // make a BBValueType constructed from a value of type that is at `index` of the variant
            // The value itself is either constructed from args or parsed from args
            // Note: index cannot be std::monostate
            // throws if variant construction fails
            assert(index != 0);
            // Note: make index sequence equal to the number of types in BBValueType
            return makeHelper(index, std::make_index_sequence<BB_VALUE_TYPE_NAMES_SIZE + 2>(), std::forward<Args>(args)...);
        }

    private:
        template <class T>
        struct Parser
        {
            auto operator()(const std::string& value) const
            {
                YAML::Node node = YAML::Load(value);
                return node.as<T>();
            }
        };

        template <class... Args, std::size_t... Is>
        static BBValueType makeHelper(std::size_t index, std::index_sequence<Is...>, Args&&... args)
        {
            std::array<BBValueType, sizeof...(Is)> vals = {makeBBValueIfIndex<Is>::make(index, std::forward<Args>(args)...)...};
            if (!vals[index].index()) {
                // variant construction failed, throw exception
                throw BlackboardException("variant construction failed");
            }
            return vals[index];
        }

        template <std::size_t INDEX>
        struct makeBBValueIfIndex
        {
            using TypeAtIndex = std::decay_t<decltype(std::get<INDEX>(std::declval<BBValueType>()))>;

            template <class... Args>
            static BBValueType make(std::size_t index, Args&&... args)
            {
                // if INDEX == index, make a variant with a value of type that is same as the variant's type at index INDEX, initialized (or parsed) with value
                // else makes a invalid variant, i.e. with value monostate
                if constexpr (PARSE_ARGS) {
                    if constexpr (!std::is_same_v<TypeAtIndex, std::monostate> && !std::is_same_v<TypeAtIndex, std::any>) {
                        return index == INDEX ? BBValueType{TypeAtIndex{Parser<TypeAtIndex>{}(args)...}} : BBValueType{};
                    }
                } else {
                    if constexpr (std::is_constructible_v<TypeAtIndex, Args&&...>) {
                        return index == INDEX ? BBValueType{TypeAtIndex{std::forward<Args>(args)...}} : BBValueType{};
                    }
                }
                return BBValueType{};
            }
        };
    };

    // trait to check if a type is one of the types in the variant
    template <class T1, class T2>
    struct isTypeInVariant;

    template <class T>
    struct isTypeInVariant<T, std::variant<>> : std::false_type
    {
    };

    template <class T, class... VariantTs>
    struct isTypeInVariant<T, std::variant<T, VariantTs...>> : std::true_type
    {
    };

    template <class T, class First, class... VariantTs>
    struct isTypeInVariant<T, std::variant<First, VariantTs...>> : isTypeInVariant<T, std::variant<VariantTs...>>
    {
    };
};

class Blackboard
{
public:
    Blackboard() = default;
    Blackboard(Blackboard&&) = delete;
    Blackboard& operator&=(const Blackboard&) = delete;
    Blackboard& operator&=(Blackboard&&) = delete;
    Blackboard(const BlackboardBlueprint* blueprint)
            : _impl{blueprint}
            , _mtx{}
    {
    }

    std::shared_lock<std::shared_mutex> lockRO() const { return std::shared_lock(_mtx); }
    std::unique_lock<std::shared_mutex> lockRW() { return std::unique_lock(_mtx); }

    // Not thread safe.  Avoid for public use
    BlackboardImpl& impl() { return _impl; }
    const BlackboardImpl& impl() const { return _impl; }

private:
    BlackboardImpl _impl;
    mutable std::shared_mutex _mtx;
};

class LockedBlackboardRO
{
public:
    LockedBlackboardRO(const Blackboard& bb)
            : _lk(bb.lockRO())
            , _impl(&bb.impl())
    {
    }
    LockedBlackboardRO& operator&=(const LockedBlackboardRO&) = delete;
    LockedBlackboardRO& operator&=(LockedBlackboardRO&) = delete;
    LockedBlackboardRO(LockedBlackboardRO&) = delete;

    bool empty() const { return _impl->empty(); }
    size_t size() const { return _impl->size(); }

    template <typename T>
    decltype(auto) get(const std::string& key) const
    {
        return _impl->get<T>(key);
    }
    bool hasValue(const std::string& key) const { return _impl->hasValue(key); }

private:
    std::shared_lock<std::shared_mutex> _lk;
    const BlackboardImpl* _impl;
};

class LockedBlackboardRW
{
public:
    LockedBlackboardRW(Blackboard& bb)
            : _lk(bb.lockRW())
            , _impl(&bb.impl())
    {
    }
    LockedBlackboardRW& operator&=(const LockedBlackboardRW&) = delete;
    LockedBlackboardRW& operator&=(LockedBlackboardRW&) = delete;
    LockedBlackboardRW(LockedBlackboardRW&) = delete;

    template <typename T>
    decltype(auto) get(const std::string& key) const
    {
        return _impl->get<T>(key);
    }
    template <typename T>
    decltype(auto) get(const std::string& key)
    {
        return _impl->get<T>(key);
    }

    template <typename T>
    void set(const std::string& key, T&& value)
    {
        _impl->set(key, std::forward<T>(value));
    }

    bool empty() const { return _impl->empty(); }
    size_t size() const { return _impl->size(); }
    bool hasValue(const std::string& key) const { return _impl->hasValue(key); }

private:
    std::unique_lock<std::shared_mutex> _lk;
    BlackboardImpl* _impl;
};

} // namespace alica
