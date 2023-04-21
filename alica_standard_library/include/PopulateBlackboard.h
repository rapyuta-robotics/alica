#pragma once

#include <boost/dll/alias.hpp>

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>

#include <array>
#include <string>
#include <tuple>

namespace alica_standard_library
{

class PopulateBlackboard : public alica::BasicPlan
{
public:
    PopulateBlackboard(alica::PlanContext& context);
    static std::unique_ptr<PopulateBlackboard> create(alica::PlanContext& context);

protected:
    void onInit() override;

private:
    using KnownTypes = std::tuple<bool, int64_t, uint64_t, double, std::string>;
    static constexpr std::array<const char*, 5> _knownTypes{"bool", "int64", "uint64", "double", "std::string"};

    void set(const std::string& key, const std::string& type);

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
            alica::UnlockedBlackboard bb{*getBlackboard()};
            bb.set(key, _json[key].as<Type>());
        }
    }

    YAML::Node _json;
};

BOOST_DLL_ALIAS(alica_standard_library::PopulateBlackboard::create, PopulateBlackboard)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PopulateBlackboardUtilityFunction)

} // namespace alica_standard_library
