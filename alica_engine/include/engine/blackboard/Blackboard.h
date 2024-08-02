#pragma once

#include "BlackboardBlueprint.h"
#include "engine/Types.h"
#include "engine/logging/Logging.h"
#include "engine/modelmanagement/Strings.h"
#include <any>
#include <array>
#include <cassert>
#include <exception>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <yaml-cpp/yaml.h>

namespace alica
{

namespace test
{
class SingleAgentBlackboardTestFixture;
class TestBlackboard;
} // namespace test

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

namespace internal
{

class BlackboardImpl
{
    using BBValueType = std::variant<std::monostate, bool, int64_t, uint64_t, double, std::string, std::any>;
    static constexpr std::size_t BB_VALUE_TYPE_ANY_INDEX = 6;

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

            if (keyInfo.type == "std::any") {
                // explicitly set the index for std::any, since getTypeIndex() returns empty for std::any
                typeIndex = BB_VALUE_TYPE_ANY_INDEX;
            }
            try {
                // insert a default constructed value or construct from the specified default value for the key
                if (keyInfo.defaultValue.empty()) {
                    _vals.emplace(
                            std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(makeBBValueForIndex<false>::make(typeIndex.value())));
                } else {
                    _vals.emplace(std::piecewise_construct, std::forward_as_tuple(key),
                            std::forward_as_tuple(makeBBValueForIndex<true>::make(typeIndex.value(), keyInfo.defaultValue)));
                }
            } catch (const std::exception& ex) {
                throw BlackboardException(stringify("Could not initialize key: ", key, " with value of type: ", keyInfo.type, ", details: ", ex.what()));
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
                if constexpr (findInVariant<T, BBValueType>::value != -1) {
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
                if constexpr (findInVariant<T, BBValueType>::value != -1) {
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
        using IntPromotedType = std::conditional_t<std::is_integral_v<Type> && !std::is_same_v<Type, bool>,
                std::conditional_t<std::is_signed_v<Type>, int64_t, uint64_t>, Type>;
        // Promote float to double. Note: we promote to double & not long double for backwards compatibility (otherwise variant type would need to be changed
        // resulting in long double being treated as a known type instead of unknown (std::any) & all get<double> statements changing to get<long double>)
        using PromotedType = std::conditional_t<std::is_same_v<IntPromotedType, float>, double, IntPromotedType>;
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

    void mapValue(const std::string& key, const std::string& value)
    {
        const std::string errorPrefix = stringify("mapConst() failure, key: ", key, ", value: ", value, ", details: ");
        try {
            _vals.at(key) = makeBBValueForIndex<true>::make(getTypeIndex(_yamlType.at(key)).value(), value);
        } catch (const std::out_of_range&) {
            throw BlackboardException(stringify(errorPrefix, "key not found in yaml file"));
        } catch (const std::bad_optional_access&) {
            throw BlackboardException(stringify(errorPrefix, "type not supported for constant mapping"));
        } catch (const BlackboardException& ex) {
            throw BlackboardException(stringify(errorPrefix, ex.what()));
        }
    }

    bool hasValue(const std::string& key) const { return _vals.count(key); }
    void removeValue(const std::string& key) { _vals.erase(key); }

    void clear() { _vals.clear(); }
    bool empty() const { return _vals.empty(); }
    size_t size() const { return _vals.size(); }

private:
    std::unordered_map<std::string, BBValueType> _vals;
    std::unordered_map<std::string, std::string> _yamlType;

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
        auto idx = findInVariant<std::decay_t<T>, BBValueType>::value;
        idx = (idx == -1) ? BB_VALUE_TYPE_ANY_INDEX : idx; // If a type is not found, treat it as std::any
        return getTypeName(idx);
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
        return std::nullopt;
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
                if constexpr (std::is_same_v<T, std::string>) {
                    // This ensures that a value that itself is a json is passed as a string instead of being parsed again
                    return value;
                }
                // TODO: using Load on value may have some pitfalls, eg. the if statement above is to deal with one of them
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

    // trait to find the index of the given type in the variant, -1 if not found
    template <class T, class Variant, std::size_t numTypes>
    struct findInVariantImpl;

    template <class T, std::size_t NUM_TYPES>
    struct findInVariantImpl<T, std::variant<>, NUM_TYPES> : std::integral_constant<std::ptrdiff_t, -1>
    {
    };

    template <class T, class... VariantTs, std::size_t NUM_TYPES>
    struct findInVariantImpl<T, std::variant<T, VariantTs...>, NUM_TYPES> : std::integral_constant<std::ptrdiff_t, NUM_TYPES - sizeof...(VariantTs) - 1>
    {
    };

    template <class T, class First, class... VariantTs, std::size_t NUM_TYPES>
    struct findInVariantImpl<T, std::variant<First, VariantTs...>, NUM_TYPES> : findInVariantImpl<T, std::variant<VariantTs...>, NUM_TYPES>
    {
    };

    template <class T1, class Variant>
    struct findInVariant;

    template <class T, class... VariantTs>
    struct findInVariant<T, std::variant<VariantTs...>> : public findInVariantImpl<T, std::variant<VariantTs...>, sizeof...(VariantTs)>
    {
    };
};

template <typename B>
class UnlockedBlackboardROImpl;
} // namespace internal

class Blackboard
{
    friend class LockedBlackboardRW;
    friend class LockedBlackboardRO;
    template <typename B>
    friend class internal::UnlockedBlackboardROImpl;
    friend class BlackboardUtil;
    friend class TestBlackboard;

public:
    Blackboard() = default;
    Blackboard(Blackboard&&) = delete;
    Blackboard& operator=(const Blackboard&) = delete;
    Blackboard& operator=(Blackboard&&) = delete;
    Blackboard(const BlackboardBlueprint* blueprint)
            : _impl{blueprint}
            , _mtx{}
    {
    }

    std::shared_lock<std::shared_mutex> lockRO() const { return std::shared_lock(_mtx); }
    std::unique_lock<std::shared_mutex> lockRW() { return std::unique_lock(_mtx); }

    // TODO: once these APIs are removed from user code, make these methods private so that only alica internal's can use them
    [[deprecated("Use UnlockedBlackboard instead")]] internal::BlackboardImpl& impl() { return _impl; }
    [[deprecated("Use UnlockedBlackboard instead")]] const internal::BlackboardImpl& impl() const { return _impl; }

private:
    friend alica::test::SingleAgentBlackboardTestFixture;
    friend alica::test::TestBlackboard;
    internal::BlackboardImpl _impl;
    mutable std::shared_mutex _mtx;
};

namespace internal
{
template <typename B = const Blackboard>
class UnlockedBlackboardROImpl
{
public:
    explicit UnlockedBlackboardROImpl(B& bb)
            : _impl(bb._impl)
    {
    }

    UnlockedBlackboardROImpl(const UnlockedBlackboardROImpl&) = delete;
    UnlockedBlackboardROImpl(UnlockedBlackboardROImpl&&) = delete;
    UnlockedBlackboardROImpl& operator=(const UnlockedBlackboardROImpl&) = delete;
    UnlockedBlackboardROImpl& operator=(UnlockedBlackboardROImpl&&) = delete;

    template <typename T>
    decltype(auto) get(const std::string& key) const
    {
        return _impl.template get<T>(key);
    }

    bool empty() const { return _impl.empty(); }
    size_t size() const { return _impl.size(); }
    bool hasValue(const std::string& key) const { return _impl.hasValue(key); }

protected:
    decltype((std::declval<B>()._impl))& _impl;
};

template <typename B = Blackboard>
class UnlockedBlackboardRWImpl : public UnlockedBlackboardROImpl<B>
{
public:
    using UnlockedBlackboardROImpl<B>::UnlockedBlackboardROImpl;

    template <typename T>
    void set(const std::string& key, T&& value)
    {
        UnlockedBlackboardRWImpl::_impl.set(key, std::forward<T>(value));
    }
};
} // namespace internal

class LockedBlackboardRO
{
public:
    LockedBlackboardRO(const Blackboard& bb)
            : _lk(bb.lockRO())
            , _impl(bb._impl)
    {
    }
    LockedBlackboardRO& operator=(const LockedBlackboardRO&) = delete;
    LockedBlackboardRO& operator=(LockedBlackboardRO&&) = delete;
    LockedBlackboardRO(const LockedBlackboardRO&) = delete;
    LockedBlackboardRO(LockedBlackboardRO&&) = delete;

    bool empty() const { return _impl.empty(); }
    size_t size() const { return _impl.size(); }

    template <typename T>
    decltype(auto) get(const std::string& key) const
    {
        return _impl.get<T>(key);
    }
    bool hasValue(const std::string& key) const { return _impl.hasValue(key); }

private:
    std::shared_lock<std::shared_mutex> _lk;
    const internal::BlackboardImpl& _impl;
};

class LockedBlackboardRW
{
public:
    LockedBlackboardRW(Blackboard& bb)
            : _lk(bb.lockRW())
            , _impl(bb._impl)
    {
    }
    LockedBlackboardRW& operator=(const LockedBlackboardRW&) = delete;
    LockedBlackboardRW& operator=(LockedBlackboardRW&&) = delete;
    LockedBlackboardRW(const LockedBlackboardRW&) = delete;
    LockedBlackboardRW(LockedBlackboardRW&&) = delete;

    template <typename T>
    decltype(auto) get(const std::string& key) const
    {
        return _impl.get<T>(key);
    }
    template <typename T>
    decltype(auto) get(const std::string& key)
    {
        return _impl.get<T>(key);
    }

    template <typename T>
    void set(const std::string& key, T&& value)
    {
        _impl.set(key, std::forward<T>(value));
    }

    bool empty() const { return _impl.empty(); }
    size_t size() const { return _impl.size(); }
    bool hasValue(const std::string& key) const { return _impl.hasValue(key); }

private:
    std::unique_lock<std::shared_mutex> _lk;
    internal::BlackboardImpl& _impl;
};

using UnlockedBlackboardRO = internal::UnlockedBlackboardROImpl<>;
using UnlockedBlackboard = internal::UnlockedBlackboardRWImpl<>;
} // namespace alica
