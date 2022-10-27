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

// using BlackboardValueType = std::variant<bool, int, std::string, std::any>;

template <class T, class = void>
struct convert
{
    static const auto& get(const BlackboardValueType& val) { return std::any_cast<const T&>(std::get<std::any>(val)); }
};

template <class T>
struct convert<T, std::enable_if_t<Find<T, BlackboardValueType>::result>>
{
    static const auto& get(const BlackboardValueType& val) { return std::get<T>(val); }
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
            return convert<T>::get(vals.at(key));
        } catch (const std::bad_variant_access& e) {
            throw BlackboardTypeMismatch(getNameFromType<T>(), getBlackboardValueType(key));
        } catch (const std::bad_any_cast& e) {
            throw BlackboardTypeMismatch(getNameFromType<T>(), getBlackboardValueType(key));
        }
    }
    template <typename T>
    const T& get(const std::string& key)
    {
        try {
            return convert<T>::get(vals.at(key));
        } catch (const std::bad_variant_access& e) {
            throw BlackboardTypeMismatch(getNameFromType<T>(), getBlackboardValueType(key));
        } catch (const std::bad_any_cast& e) {
            throw BlackboardTypeMismatch(getNameFromType<T>(), getBlackboardValueType(key));
        }
    }
    BlackboardValueType& get(const std::string& key) { return vals.at(key); }
    const BlackboardValueType& get(const std::string& key) const { return vals.at(key); }

    template <class T>
    void set(const std::string& key, const T& value)
    {
        if (getBlackboardValueNode(key).Type() == YAML::NodeType::Null) {
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
            std::string pmlType = getBlackboardValueType(key);
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

    bool hasValue(const std::string& key) const { return vals.count(key); }
    void removeValue(const std::string& key) { vals.erase(key); }

    void initDefaultValues();

    void clear() { vals.clear(); }
    bool empty() const { return vals.empty(); }
    size_t size() const { return vals.size(); }
    std::unordered_map<std::string, BlackboardValueType> vals;
    YAML::Node node;

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
        YAML::Node entry = getBlackboardValueNode(key);
        return entry[Strings::stateType].as<std::string>();
    }

    template <class T>
    std::string getNameFromType() const;
};

struct Converter
{
    static constexpr const char* typeNames[] = {"bool", "int64", "double", "std::string", "std::any"};

    // YAML doesnt support std::any, needs a separate list of types for iteration
    static constexpr const char* typeNamesYAML[] = {"bool", "int64", "double", "std::string"};
    using TypesYAML = std::variant<bool, int64_t, double, std::string>;

    // convert from type to name as string
    template <class T>
    static constexpr const char* typeName()
    {
        return typeNames[BlackboardValueType{T{}}.index()];
    }

    // set the value given the type name as a string
    static void setValueFromYaml(const std::string& key, const std::string& typeName, const YAML::Node& valueNode, BlackboardImpl& bb);

    // used to set the value without a yaml node
    static void setValue(const std::string& key, const BlackboardValueType& value, const std::string& typeName, BlackboardImpl& bb);

    template <class T>
    struct YamlAs
    {
        static T as(const YAML::Node& value) { return value.as<T>(); }
    };

    template <class T, std::size_t INDEX, template <class> class Parser = YamlAs>
    struct makeVariantIfEqual
    {
        static BlackboardValueType make(std::size_t index, const YAML::Node& value)
        {
            return index == INDEX ? BlackboardValueType{Parser<T>::as(value)} : BlackboardValueType{std::any{}};
        }
    };

    template <class... Ts, std::size_t... Is>
    static BlackboardValueType makeVariantFromIndex(std::variant<Ts...>, std::index_sequence<Is...>, std::size_t index, const YAML::Node& value)
    {
        BlackboardValueType typeValues[] = {makeVariantIfEqual<Ts, Is>::make(index, value)...};
        return typeValues[index];
    };

    template <class T, std::size_t INDEX, template <class> class Parser = YamlAs>
    struct makeAnyIfEqual
    {
        static std::any make(std::size_t index, const YAML::Node& value) { return index == INDEX ? std::any{Parser<T>::as(value)} : std::any{}; }
    };

    template <class... Ts, std::size_t... Is>
    static std::any makeAnyFromIndex(std::variant<Ts...>, std::index_sequence<Is...>, std::size_t index, const YAML::Node& value)
    {
        std::any typeValues[] = {makeAnyIfEqual<Ts, Is>::make(index, value)...};
        return typeValues[index];
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
    const T& get(const std::string& key)
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
