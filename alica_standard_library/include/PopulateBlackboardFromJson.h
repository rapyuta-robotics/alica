#pragma once

#include <boost/dll/alias.hpp>

#include <engine/BasicBehaviour.h>

#include <array>
#include <string>
#include <tuple>

namespace alica_standard_library
{

class PopulateBlackboardFromJson : public alica::BasicBehaviour
{
public:
    PopulateBlackboardFromJson(alica::BehaviourContext& context);
    static std::unique_ptr<PopulateBlackboardFromJson> create(alica::BehaviourContext& context);

protected:
    void initialiseParameters() override;

private:
    using KnownTypes = std::tuple<bool, int64_t, uint64_t, double, std::string>;
    static constexpr std::array<const char*, 5> _knownTypes{"bool", "int64", "uint64", "double", "std::string"};

    struct Context
    {
        YAML::Node jsonNode;
        const alica::BlackboardBlueprint* blueprint;
        std::vector<std::string> blackboardKeys;
        std::shared_ptr<alica::Blackboard> blackboard;
    } _context;

    void readContext();
    std::string verifyContext();
    void populateFromContext();

    void setError(const std::string& error);

    template <std::size_t... Is>
    void setHelper(const std::string& key, std::size_t typeIndex, std::index_sequence<Is...>)
    {
        struct Dummy
        {
            static void dummyFunc(...) {}
        };
        Dummy::dummyFunc((setIfIndex<Is>(key, typeIndex), 0)...);
    }

    template <std::size_t TYPE_INDEX>
    void setIfIndex(const std::string& key, std::size_t typeIndex)
    {
        if (typeIndex == TYPE_INDEX) {
            using Type = std::decay_t<decltype(std::get<TYPE_INDEX>(std::declval<KnownTypes>()))>;
            alica::LockedBlackboardRW bb{*(_context.blackboard)};
            bb.set(key, _context.jsonNode[key].as<Type>());
        }
    }
};

BOOST_DLL_ALIAS(alica_standard_library::PopulateBlackboardFromJson::create, PopulateBlackboardFromJson)

} // namespace alica_standard_library
