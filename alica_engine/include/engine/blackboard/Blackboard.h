#pragma once

#include "BlackboardBlueprint.h"
#include "engine/Types.h"
#include "engine/modelmanagement/Strings.h"
#include <any>
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

class BlackboardImpl
{
public:
    // Not for API use, but public to allow modifying without mutex when we know the behavior/plan is not running
    template <class... Args>
    void registerValue(const std::string& key, Args&&... args)
    {
        _vals.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(std::forward<decltype(args)>(args)...));
    }

    template <typename T>
    const T& get(const std::string& key) const
    {
        try {
            const auto yamlTypeIt = _yamlType.find(key);
            if (yamlTypeIt != _yamlType.end() && yamlTypeIt->second == "std::any") {
                // yaml type is std::any, so would be stored as std::any
                return std::any_cast<const T&>(std::get<std::any>(_vals.at(key)));
            }
            if constexpr (isTypeInVariant<std::decay_t<T>, BlackboardValueType>::value) {
                // T is a known type, directly use std::get to fetch from variant
                // T must be an exact match to some type in the variant, type conversions are not taken into account
                return std::get<T>(_vals.at(key));
            } else {
                // T is an unknown type, use std::any for variant type & any_cast to T
                // T must be an exact match to the T used while setting this key (i.e. no type conversions taken into account)
                // Exact matches are required because we return by reference
                return std::any_cast<const T&>(std::get<std::any>(_vals.at(key)));
            }
        } catch (const std::bad_variant_access& e) {
            // TODO: raise exception
        } catch (const std::bad_any_cast& e) {
            // TODO: raise exception
        } catch (const std::out_of_range& e) {
            // TODO: raise exception
        }
    }

    template <typename T>
    T& get(const std::string& key)
    {
        return const_cast<T&>(static_cast<const BlackboardImpl*>(this)->get<T>(key));
    }
    BlackboardValueType& get(const std::string& key) { return _vals.at(key); }
    const BlackboardValueType& get(const std::string& key) const { return _vals.at(key); }

    template <class T>
    void set(const std::string& key, T&& value)
    {
        const char* type = getTypeName<T>();
        const auto yamlTypeIt = _yamlType.find(key);
        if (yamlTypeIt == _yamlType.end()) {
            // key is not found in yaml file
            if (!type) {
                // unknown type, use std::any
                _values[key] = std::any{std::forward<T>(value)};
            } else {
                // known type, just assign (type conversions are taken into account)
                _values[key] = std::forward<T>(value);
            }
        } else {
            // key is found in yaml file
            const auto& yamlType = yamlTypeIt->second;
            if (yamlType == "std::any") {
                // yaml type is std::any, so can store values of any type, therefore use std::any for variant type
                _values[key] = std::any{std::forward<T>(value)};
            } else {
                if (!type || yamlType != type) {
                    // TODO: raise exception
                } else {
                    // types match, assign
                    _values[key] = std::forward<T>(value);
                }
            }
        }
    }

    void map(const std::string& srcKey, const std::string& targetKey, const BlackboardImpl& srcBb)
    {
        // Note: srcKey & targetKey has to be set
        // mapping succeeds as long as targetType is constructible from srcType (so conversions are supported)
        // TODO: Fix compiler warning
        std::visit(
                [srcKey, targetKey, this](auto&& srcValue) {
                    // set the target as a side effect, throws if targetType is not constructible from srcType
                    _vals[targetKey] = makeBBValueForIndex<false>::make(_vals[targetKey].index(), std::forward<decltype(srcValue)>(srcValue));
                },
                srcBb._vals.at(srcKey));
    }

    bool hasValue(const std::string& key) const { return _vals.count(key); }
    void removeValue(const std::string& key) { _vals.erase(key); }

    void initDefaultValues();

    void clear() { _vals.clear(); }
    bool empty() const { return _vals.empty(); }
    size_t size() const { return _vals.size(); }
    std::unordered_map<std::string, BlackboardValueType> _vals;
    YAML::Node node;
    std::unordered_map<std::string, std::string> _yamlType;

private:
    friend BlackboardUtil;

    std::string getBlackboardValueType(const std::string& key) const { return _yamlType.at(key); }

    template <class T>
    std::string getTypeName() const;

    static constexpr const char* BB_VALUE_TYPE_NAMES[] = {"bool", "int64", "double", "std::string", "std::any"};
    static constexpr std::size_t BB_VALUE_TYPE_NAMES_SIZE = sizeof(BB_VALUE_TYPE_NAMES) / sizeof(const char*);

    // Used to create a variant of type BlackboardValueType from an index
    template <bool PARSE_ARGS = false>
    struct makeBBValueForIndex
    {
        using KnownTypes = std::variant<bool, int64_t, double, std::string>;

        template <class T>
        struct Parser
        {
            auto operator()(const std::string& value) const
            {
                if constexpr (Find<T, KnownTypes>::result) {
                    // type is a known type, continue parsing
                    YAML::Node node = YAML::Load(value);
                    return node.as<T>();
                } else {
                    // type is not a known type, throw exception
                    throw BlackboardTypeMismatch("", "<bool, int64, double, std::string>");
                    return T{};
                }
            }
        };

        template <class... Args>
        static BlackboardValueType make(std::size_t index, Args&&... args)
        {
            // BB_VALUE_TYPE_NAMES_SIZE + 1 to account for std::monostate
            return makeHelper(index, std::make_index_sequence<BB_VALUE_TYPE_NAMES_SIZE + 1>(), std::forward<Args>(args)...);
        }

        template <class... Args, std::size_t... Is>
        static BlackboardValueType makeHelper(std::size_t index, std::index_sequence<Is...>, Args&&... args)
        {
            BlackboardValueType _vals[] = {makeBBValueIfIndex<Is>::make(index, std::forward<Args>(args)...)...};
            if (!_vals[index].index()) {
                // TODO: variant construction failed, throw exception, remove return
                return _vals[index];
            }
            return _vals[index];
        }

        template <std::size_t INDEX>
        struct makeBBValueIfIndex
        {
            using TypeAtIndex = std::decay_t<decltype(std::get<INDEX>(std::declval<BlackboardValueType>()))>;

            template <class... Args>
            static BlackboardValueType make(std::size_t index, Args&&... args)
            {
                // if INDEX == index, make a variant with a value of type that is same as the variant's type at index INDEX, intialized (or parsed) with value
                // else makes a invalid variant, i.e. with value monostate
                if constexpr (PARSE_ARGS) {
                    return index == INDEX ? BlackboardValueType{TypeAtIndex{Parser<TypeAtIndex>{}(args)...}} : BlackboardValueType{};
                } else {
                    if constexpr (std::is_constructible_v<TypeAtIndex, Args&&...>) {
#pragma warning(suppress : 4101) // TODO: fix compiler warning without suppression
                        return index == INDEX ? BlackboardValueType{TypeAtIndex{std::forward<Args>(args)...}} : BlackboardValueType{};
                    } else {
                        return BlackboardValueType{};
                    }
                }
            }
        };
    };

    struct Converter
    {
        // static constexpr const char* typeNames[] = {"bool", "int64", "double", "std::string", "std::any"};

        // convert from type to name as string
        template <class T>
        static constexpr const char* typeName()
        {
            return BB_VALUE_TYPE_NAMES[BlackboardValueType{T{}}.index() - 1]; // account for monostate in variant
        }
    };

    template <class T1, class T2>
    struct isTypeInVariant;

    template <class T>
    struct isTypeInVariant<T, std::variant<>> : std::false_type {};

    template <class T, class... VariantTs>
    struct isTypeInVariant<T, std::variant<T, VariantTs...>> : std::true_type {};

    template <class T, class First, class... VariantTs>
    struct isTypeInVariant<T, std::variant<First, VariantTs...>> : isTypeInVariant<T, std::variant<VariantTs...>> {};
};

class Blackboard
{
public:
    Blackboard() = default;
    Blackboard(Blackboard&&) = delete;
    Blackboard& operator&=(const Blackboard&) = delete;
    Blackboard& operator&=(Blackboard&&) = delete;
    Blackboard(const BlackboardBlueprint* blueprint)
    {
        _impl._vals = blueprint->_vals;
        _impl.node = blueprint->node;
        // store blackboard keys from pml file with their type in a map
        for (const auto& entry : _impl.node) {
            std::string key = entry[Strings::key].as<std::string>();
            std::string type = entry[Strings::stateType].as<std::string>();
            _impl._yamlType[key] = type;
        }
        _impl.initDefaultValues();
    };

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
    const T& get(const std::string& key) const
    {
        return _impl->get<T>(key);
    }
    const BlackboardValueType& get(const std::string& key) const { return _impl->get(key); }
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

    template <class... Args>
    [[deprecated]] void registerValue(const std::string& key, Args&&... args)
    {
        _impl->registerValue(key, std::forward<Args>(args)...);
    }
    template <typename T>
    const T& get(const std::string& key) const
    {
        return _impl->get<T>(key);
    }
    template <typename T>
    T& get(const std::string& key)
    {
        return _impl->get<T>(key);
    }
    BlackboardValueType& get(const std::string& key) { return _impl->get(key); }

    template <typename T>
    void set(const std::string& key, const T& value)
    {
        _impl->set<T>(key, value);
    }

    bool empty() const { return _impl->empty(); }
    size_t size() const { return _impl->size(); }
    bool hasValue(const std::string& key) const { return _impl->hasValue(key); }

private:
    std::unique_lock<std::shared_mutex> _lk;
    BlackboardImpl* _impl;
};

template <class T>
std::string BlackboardImpl::getTypeName() const
{
    return Converter::typeName<T>();
}

} // namespace alica
