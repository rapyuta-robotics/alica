#pragma once

#include "BlackboardBlueprint.h"
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

class BlackboardImpl;

struct Converter
{
    static constexpr const char* typeNames[] = {"bool", "int64", "double", "std::string"};
    using Types = std::variant<bool, int64_t, double, std::string>;

    // convert from type to name as string
    template <class T>
    static constexpr const char* typeName()
    {
        return typeNames[Types{T{}}.index()];
    }

    // set the value given the type name as a string
    static void setDefaultValue(const std::string& key, const std::string& typeName, const YAML::Node& defaultValue, BlackboardImpl& bb);

    template <class T>
    struct YamlAs
    {
        static T as(const YAML::Node& value) { return value.as<T>(); }
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

class BlackboardTypeMismatch : public std::exception
{
public:
    BlackboardTypeMismatch(const std::string& type, const std::string& typeDefinedInPml)
            : _inputType(type)
            , _typeDefinedInPml(typeDefinedInPml)
    {
    }

    const char* what() const throw() override { return "The type of the blackboard value does not match the type in the pml file."; }

    std::string detailedError()
    {
        std::stringstream ss;
        ss << "Type " << _inputType << " does not match the type defined in the pml file: " << _typeDefinedInPml;
        return ss.str();
    }

private:
    std::string _inputType;
    std::string _typeDefinedInPml;
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
            return std::any_cast<const T&>(vals.at(key));
        } catch (const std::bad_any_cast& e) {
            throw BlackboardTypeMismatch(getNameFromType<T>(), getBlackboardValueType(key));
        }
    }
    template <typename T>
    T& get(const std::string& key)
    {
        try {
            return std::any_cast<T&>(vals.at(key));
        } catch (const std::bad_any_cast& e) {
            throw BlackboardTypeMismatch(getNameFromType<T>(), getBlackboardValueType(key));
        }
    }
    std::any& get(const std::string& key) { return vals.at(key); }
    const std::any& get(const std::string& key) const { return vals.at(key); }

    template <class T>
    void set(const std::string& key, const T& value)
    {
        YAML::Node entry = getBlackboardValueNode(key);
        if (entry[Strings::key].as<std::string>() != key) {
            // value is not predefined in pml file
            vals.at(key) = value;
        } else {
            // value is predefined in pml file
            std::string inputType = getNameFromType<T>();
            std::string pmlType = getBlackboardValueType(key);
            if (pmlType == inputType) {
                vals.at(key) = value;
            } else {
                // types dont match, throw exception
                throw BlackboardTypeMismatch(inputType, pmlType);
            }
        }
    }

    bool hasValue(const std::string& key) const { return vals.count(key); }
    void removeValue(const std::string& key) { vals.erase(key); }

    void initDefaultValues()
    {
        for (YAML::Node entry : node) {
            YAML::Node defaultValue = entry[Strings::defaultValue];
            if (defaultValue.Type() != YAML::NodeType::Null) {
                std::string key = entry[Strings::key].as<std::string>();
                std::string typeString = entry[Strings::stateType].as<std::string>();
                Converter::setDefaultValue(key, typeString, defaultValue, *this);
            }
        }
    }

    void clear() { vals.clear(); }
    bool empty() const { return vals.empty(); }
    size_t size() const { return vals.size(); }
    std::unordered_map<std::string, std::any> vals;
    YAML::Node node;

private:
    YAML::Node getBlackboardValueNode(std::string key) const
    {
        for (const auto& entry : node) {
            if (entry[Strings::key].as<std::string>() == key) {
                return entry;
            }
        }
        return YAML::Node();
    }

    std::string getBlackboardValueType(const std::string& key) const
    {
        YAML::Node entry = getBlackboardValueNode(key);
        return entry[Strings::stateType].as<std::string>();
    }

    template <class T>
    std::string getNameFromType() const
    {
        if (std::is_same<T, int64_t>::value) {
            return "int64_t";
        } else if (std::is_same<T, bool>::value) {
            return "bool";
        } else if (std::is_same<T, double>::value) {
            return "double";
        } else if (std::is_same<T, std::string>::value) {
            return "std::string";
        }
        // treat as std::any by default
        return "std::any";
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
    const std::any& get(const std::string& key) const { return _impl->get(key); }
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
    T& get(const std::string& key)
    {
        return _impl->get<T>(key);
    }
    std::any& get(const std::string& key) { return _impl->get(key); }

    void set(const std::string& key, const std::any& value) { _impl->set(key, value); }

    bool empty() const { return _impl->empty(); }
    size_t size() const { return _impl->size(); }
    bool hasValue(const std::string& key) const { return _impl->hasValue(key); }

private:
    std::unique_lock<std::shared_mutex> _lk;
    BlackboardImpl* _impl;
};

} // namespace alica
