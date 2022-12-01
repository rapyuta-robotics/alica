#pragma once

#include "BlackboardBlueprint.h"
#include "engine/Types.h"
#include "engine/modelmanagement/Strings.h"
#include <any>
#include <cassert>
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
    using BBValueType = std::variant<std::monostate, bool, int64_t, double, std::string, std::any>;
    static constexpr std::size_t BB_VALUE_TYPE_ANY_INDEX = 5;

public:
    BlackboardImpl() = default;
    BlackboardImpl(const BlackboardImpl&) = delete;
    BlackboardImpl(const BlackboardBlueprint* blueprint)
    {
        for (auto it = blueprint->begin(); it != blueprint->end(); ++it) {
            const auto& [key, keyInfo] = *it;
            _yamlType.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(keyInfo.type));
            if (keyInfo.defaultValue.has_value()) {
                auto typeIndex = getTypeIndex(keyInfo.type);
                if (!typeIndex) {
                    // unknown type, can't set default value, TODO: throw exception
                } else {
                    _vals.emplace(std::piecewise_construct, std::forward_as_tuple(key),
                            std::forward_as_tuple(makeBBValueForIndex<true>::make(typeIndex.value(), keyInfo.defaultValue.value())));
                }
            } else {
                auto typeIndex = getTypeIndex(keyInfo.type);
                if (keyInfo.type == "std::any") {
                    // getTypeIndex() does not consider std::any
                    typeIndex = BB_VALUE_TYPE_ANY_INDEX;
                }
                // insert a default constructed value
                _vals.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(makeBBValueForIndex<false>::make(typeIndex.value())));
            }
        }
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
            if constexpr (isTypeInVariant<T, BBValueType>::value) {
                // T is a known type, directly use std::get to fetch from variant
                // T must be an exact match to some type in the variant (i.e. no type conversions taken into account)
                return std::get<T>(_vals.at(key));
            } else {
                // T is an unknown type, use std::any for variant type & any_cast to T
                // T must be an exact match to the T used while setting this key (i.e. no type conversions taken into account)
                // Exact matches are required because we return by reference
                return std::any_cast<const T&>(std::get<std::any>(_vals.at(key)));
            }
        } catch (const std::bad_variant_access& e) {
            // known type, but type mismatch, TODO: raise exception
        } catch (const std::bad_any_cast& e) {
            // type mismatch (unknown type or yaml type is std::any), TODO: raise exception
        } catch (const std::out_of_range& e) {
            // key not set, TODO: raise exception
        }
    }

    template <typename T>
    T& get(const std::string& key)
    {
        return const_cast<T&>(static_cast<const BlackboardImpl*>(this)->get<T>(key));
    }
    BBValueType& get(const std::string& key) { return _vals.at(key); }
    const BBValueType& get(const std::string& key) const { return _vals.at(key); }

    template <class T>
    void set(const std::string& key, T&& value)
    {
        const char* type = getTypeName<T>();
        const auto yamlTypeIt = _yamlType.find(key);
        if (yamlTypeIt == _yamlType.end()) {
            // key is not found in yaml file
            if (!type) {
                // unknown type, use std::any
                _vals[key] = std::any{std::forward<T>(value)};
            } else {
                // known type, just assign (type conversions are taken into account)
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
                    // unknown type, TODO: raise exception
                } else if (yamlType != type) {
                    // known type, but type mismatch, TODO: raise exception
                } else {
                    // known type & types match, assign
                    _vals[key] = std::forward<T>(value);
                }
            }
        }
    }

    void map(const std::string& srcKey, const std::string& targetKey, const BlackboardImpl& srcBb)
    {
        // Note: srcKey & targetKey has to be set & both have to be found in the yaml (pml or beh) file
        assert(_vals.count(targetKey));
        assert(srcBb._vals.count(srcKey));
        // mapping succeeds as long as targetType is constructible from srcType (so conversions are supported)
        std::visit(
                [srcKey, targetKey, this](auto&& srcValue) {
                    // set the target as a side effect, throws if targetType is not constructible from srcType
                    _vals[targetKey] = makeBBValueForIndex<false>::make(_vals[targetKey].index(), std::forward<decltype(srcValue)>(srcValue));
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
    static const char* getTypeName()
    {
        auto typeIndex = BBValueType{T{}}.index() - 1;
        if (typeIndex < BB_VALUE_TYPE_NAMES_SIZE) {
            // known type (after conversions to supported types in the the variant are taken into account)
            return BB_VALUE_TYPE_NAMES[typeIndex];
        } else {
            // unknown type, note: if index is deduced as std::any, it is also treated as unknown type
            return nullptr;
        }
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

    static constexpr const char* BB_VALUE_TYPE_NAMES[] = {"bool", "int64", "double", "std::string"};
    static constexpr std::size_t BB_VALUE_TYPE_NAMES_SIZE = sizeof(BB_VALUE_TYPE_NAMES) / sizeof(const char*);

    template <bool PARSE_ARGS = false>
    struct makeBBValueForIndex
    {
        template <class... Args>
        static BBValueType make(std::size_t index, Args&&... args)
        {
            // make a BBValueType constructed from a value of type that is at `index` of the variant
            // The value itself is either constructed from args or parsed from args
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
            BBValueType vals[] = {makeBBValueIfIndex<Is>::make(index, std::forward<Args>(args)...)...};
            if (!vals[index].index()) {
                // TODO: variant construction failed, throw exception, remove return
                return vals[index];
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
                // if INDEX == index, make a variant with a value of type that is same as the variant's type at index INDEX, intialized (or parsed) with value
                // else makes a invalid variant, i.e. with value monostate
                if constexpr (PARSE_ARGS) {
                    if constexpr (isTypeInVariant<TypeAtIndex, BBValueType>::value && !std::is_same_v<TypeAtIndex, std::monostate> &&
                                  !std::is_same_v<TypeAtIndex, std::any>) {
                        return index == INDEX ? BBValueType{TypeAtIndex{Parser<TypeAtIndex>{}(args)...}} : BBValueType{};
                    }
                } else {
// TODO: figure out a way to disable warnings, caused due to std::visit. Note: really tricky since small changes can cause unexpected behaviour, so best to
// disable the warning for now
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
                    if constexpr (std::is_constructible_v<TypeAtIndex, Args&&...>) {
                        return index == INDEX ? BBValueType{TypeAtIndex{std::forward<Args>(args)...}} : BBValueType{};
                    }
#pragma GCC diagnostic pop
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
    const T& get(const std::string& key) const
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
    const T& get(const std::string& key) const
    {
        return _impl->get<T>(key);
    }
    template <typename T>
    T& get(const std::string& key)
    {
        return _impl->get<T>(key);
    }

    template <typename T>
    void set(const std::string& key, T&& value)
    {
        _impl->set<T>(key, std::forward<T>(value));
    }

    bool empty() const { return _impl->empty(); }
    size_t size() const { return _impl->size(); }
    bool hasValue(const std::string& key) const { return _impl->hasValue(key); }

private:
    std::unique_lock<std::shared_mutex> _lk;
    BlackboardImpl* _impl;
};

} // namespace alica
