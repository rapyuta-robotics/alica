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

class BlackboardTypeMismatch : public std::exception
{
public:
    BlackboardTypeMismatch(const std::string& type, const std::string& typeDefinedInPml)
            : _inputType(type)
            , _typeDefinedInPml(typeDefinedInPml)
    {
    }

    const char* what() const noexcept override { return "The type of the blackboard value does not match the type in the pml file."; }

    std::string detailedError() const
    {
        std::stringstream ss;
        ss << "Type " << _inputType << " does not match the type defined in the pml file: " << _typeDefinedInPml;
        return ss.str();
    }

private:
    const std::string _inputType;
    const std::string _typeDefinedInPml;
};

template <class Val, class... Ts>
struct FindImpl
{
    static constexpr bool result = false;
};

template <class Val, class T, class... Ts>
struct FindImpl<Val, T, Ts...>
{
    static constexpr bool result = FindImpl<Val, Ts...>::result;
};

template <class Val, class... Ts>
struct FindImpl<Val, Val, Ts...>
{
    static constexpr bool result = true;
};

template <class Val, class Variant>
struct Find;

template <class Val, template <class...> class Tp, class... Ts>
struct Find<Val, Tp<Ts...>> : FindImpl<Val, Ts...>
{
};

static constexpr const char* BB_VALUE_TYPE_NAMES[] = {"bool", "int64", "double", "std::string", "std::any"};
static constexpr std::size_t BB_VALUE_TYPE_NAMES_SIZE = sizeof(BB_VALUE_TYPE_NAMES) / sizeof(const char*);

template <bool PARSE_ARGS = false>
struct makeBBValueForIndex
{
    using KnownTypes = std::variant<bool, int64_t, double, std::string>;

    template <class T>
    struct Parser
    {
        auto operator()(const std::string& value) const
        {
            if constexpr(Find<T, KnownTypes>::result) {
                // type is a known type, continue parsing
                YAML::Node node = YAML::Load(value);
                return node.as<T>();
            } else {
                // type is not a known type, throw exception
                throw BlackboardTypeMismatch("<std::monostate, std::any>", "<bool, int64, double, std::string>");
                return T{};
            }
        }
    };

    template <class... Args>
    static BlackboardValueType make(std::size_t index, Args&&... args)
    {
        return makeHelper(index, std::make_index_sequence<BB_VALUE_TYPE_NAMES_SIZE + 1>(), std::forward<Args>(args)...);
    }

    template <class... Args, std::size_t... Is>
    static BlackboardValueType makeHelper(std::size_t index, std::index_sequence<Is...>, Args&&... args)
    {
        BlackboardValueType vals[] = {makeBBValueIfIndex<Is>::make(index, std::forward<Args>(args)...)...};
        if (!vals[index].index()) {
            // TODO: variant construction failed, throw exception, remove return
            return vals[index];
        }
        return vals[index];
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
#pragma warning(suppress: 4101) // TODO: fix compiler warning without suppression
                    return index == INDEX ? BlackboardValueType{TypeAtIndex{std::forward<Args>(args)...}} : BlackboardValueType{};
                } else {
                    return BlackboardValueType{};
                }
            }
        }
    };
};

class BlackboardImpl
{
public:
    // Not for API use, but public to allow modifying without mutex when we know the behavior/plan is not running
    template <class... Args>
    void registerValue(const std::string& key, Args&&... args)
    {
        vals.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(std::forward<decltype(args)>(args)...));
    }

    template <typename T>
    const T& get(const std::string& key) const
    {
        try {
            if constexpr (Find<T, BlackboardValueType>::result) {
                return std::get<T>(vals.at(key));
            } else {
                return std::any_cast<T>(vals.at(key));
            }
        } catch (const std::bad_variant_access& e) {
            throw BlackboardTypeMismatch(getNameFromType<T>(), getBlackboardValueType(key));
        } catch (const std::bad_any_cast& e) {
            throw BlackboardTypeMismatch(getNameFromType<T>(), getBlackboardValueType(key));
        }
    }

    template <typename T>
    T& get(const std::string& key)
    {
        return const_cast<T&>(const_cast<const BlackboardImpl*>(this)->get<T>(key));
    }
    BlackboardValueType& get(const std::string& key) { return vals.at(key); }
    const BlackboardValueType& get(const std::string& key) const { return vals.at(key); }

    template <class T>
    void set(const std::string& key, const T& value)
    {
        if (getBlackboardValueNode(key).Type() == YAML::NodeType::Null) { // replace with check if value is in keyToType map
            // value is not predefined in pml file
            if (getNameFromType<T>() == "std::any") {
                // if type is std::any, wrap into std::any
                vals.emplace(key, std::any{value});
            } else {
                vals.emplace(key, value);
            }
        } else {
            // value is predefined in pml file
            std::string inputType = getNameFromType<T>();
            std::string pmlType = keyToType.at(key);
            if (pmlType == "std::any") {
                // insert as std::any with type T
                vals.at(key) = std::any{value};
            } else if (pmlType == inputType) {
                vals.at(key) = value;
            } else {
                // types dont match, throw exception
                throw BlackboardTypeMismatch(inputType, pmlType);
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
                    vals[targetKey] = makeBBValueForIndex<false>::make(vals[targetKey].index(), std::forward<decltype(srcValue)>(srcValue));
                },
                srcBb.vals.at(srcKey));
    }

    bool hasValue(const std::string& key) const { return vals.count(key); }
    void removeValue(const std::string& key) { vals.erase(key); }

    void initDefaultValues();

    void clear() { vals.clear(); }
    bool empty() const { return vals.empty(); }
    size_t size() const { return vals.size(); }
    std::unordered_map<std::string, BlackboardValueType> vals;
    YAML::Node node;
    std::unordered_map<std::string, std::string> keyToType;

private:
    friend BlackboardUtil;
    YAML::Node getBlackboardValueNode(std::string key) const
    {
        for (const auto& entry : node) {
            if (entry[Strings::key].as<std::string>() == key) {
                return entry;
            }
        }
        return YAML::Node(YAML::NodeType::Null);
    }

    std::string getBlackboardValueType(const std::string& key) const
    {
        return keyToType.at(key);
    }

    template <class T>
    std::string getNameFromType() const;
};

struct Converter
{
    static constexpr const char* typeNames[] = {"bool", "int64", "double", "std::string", "std::any"};

    // convert from type to name as string
    template <class T>
    static constexpr const char* typeName()
    {
        return typeNames[BlackboardValueType{T{}}.index() - 1]; // account for monostate in variant
    }
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
        _impl.vals = blueprint->vals;
        _impl.node = blueprint->node;
        for (const auto& entry : _impl.node) {
            std::string key = entry[Strings::key].as<std::string>();
            std::string type = entry[Strings::stateType].as<std::string>();
            // add to map, <key, type>
            _impl.keyToType.at(key) = type;
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
std::string BlackboardImpl::getNameFromType() const
{
    return Converter::typeName<T>();
}

} // namespace alica
