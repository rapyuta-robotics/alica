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

class BlackboardUtil;

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

    void initDefaultValues();

    void clear() { vals.clear(); }
    bool empty() const { return vals.empty(); }
    size_t size() const { return vals.size(); }
    std::unordered_map<std::string, std::any> vals;
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
        return YAML::Node();
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
    using Types = std::variant<bool, int64_t, double, std::string, std::any>;

    // YAML doesnt support std::any, needs a separate list of types for iteration
    static constexpr const char* typeNamesYAML[] = {"bool", "int64", "double", "std::string"};
    using TypesYAML = std::variant<bool, int64_t, double, std::string>;

    // convert from type to name as string
    template <class T>
    static constexpr const char* typeName() {
        return typeNames[Types{T{}}.index()];
    }

    // set the value given the type name as a string
    static void setDefaultValue(const std::string& key, const std::string& typeName, const YAML::Node& defaultValue, BlackboardImpl& bb);

    // used to set the value without a yaml node
    template <class T>
    static void setValue(const std::string& key, const T& value, const std::string& typeName, BlackboardImpl& bb);

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
    };

    template <class T, std::size_t INDEX>
    struct setAnyIfEqual
    {
        static bool set(std::size_t index, const std::string& key, const std::any& value, BlackboardImpl& bb)
        {
            if (index != INDEX) {
                return false;
            }
            const T& val = std::any_cast<const T&>(value);
            bb.set<T>(key, val);
            return true;
        }
    };

    /**
    * Set an std::any value on the blackboard. The std::any value will be cast with any_cast to the correct type
    * before setting it on the blackboard.
    */
    template <class... Ts, std::size_t... Is>
    static void setBlackboardValue(std::variant<Ts...>, std::index_sequence<Is...>, std::size_t index, const std::string& key, const std::any& value, BlackboardImpl& bb)
    {
        bool vals[] = {setAnyIfEqual<Ts, Is>::set(index, key, value, bb)...};
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

    template <typename T>
    void set(const std::string& key, const T& value) { _impl->set<T>(key, value); }

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

template <class T>
void Converter::setValue(const std::string& key, const T& value, const std::string& typeName, BlackboardImpl& bb)
{
    if (typeName == "std::any") {
        bb.set<std::any>(key, value);
        return;
    }
    static constexpr std::size_t numTypes = sizeof(typeNamesYAML) / sizeof(const char*);
    for (std::size_t i = 0; i < numTypes; ++i) {
        if (typeName == typeNamesYAML[i]) {
            setBlackboardValue(TypesYAML{}, std::make_index_sequence<numTypes>{}, i, key, std::any{value}, bb);
            break;
        }
    }
}

} // namespace alica
