#pragma once


#include <functional>
#include <memory>
#include <map>
#include <string>
#include <iostream>

namespace alica
{

class Behaviour;
class AlicaEngine;

#define BEHAVIOURREGISTER_DEC_TYPE(CLASS) static DerivedBehaviourRegister<CLASS> reg_;
#define BEHAVIOURREGISTER_DEF_TYPE(CLASS, NAME) DerivedBehaviourRegister<CLASS> CLASS::reg_(NAME);

using creationFunction = std::function<std::shared_ptr<Behaviour>(AlicaEngine* ae)>;
using creationFuncDepot = std::map<std::string, creationFunction>;

class BehaviourRegister
{
public:
    static creationFuncDepot &getMap()
    {
        static creationFuncDepot map_;
        return map_;
    }

public:
    static std::function<std::shared_ptr<Behaviour>(AlicaEngine* ae)> getCreatorFunction(const std::string &commandname)
    {
        creationFunction func;
        creationFuncDepot &mymap = getMap();
        if (mymap.find(commandname) != mymap.end())
            func = mymap[commandname];
        else
            std::cerr<<"Error" << "Function creator not found for:" << commandname << std::endl;
        return func;
    }

    void Dump()
    {
        std::cerr<<"Info" << "--------" << std::endl;
        creationFuncDepot &mymap = getMap();
        for (auto current : mymap)
        {
            std::cerr<<"Info" << "--->" << current.first << std::endl;
        }
        std::cerr<<"Info" << "--------" << std::endl;
    }

    virtual ~BehaviourRegister()
    {
        creationFuncDepot &mymap = getMap();
        mymap.clear();
    };
};

template <typename T>
class DerivedBehaviourRegister : public BehaviourRegister
{
public:
    explicit DerivedBehaviourRegister(const std::string &commandname)
    {
        auto x = [](AlicaEngine* ae) {
            return std::make_shared<T>(ae);
        };

        creationFuncDepot &mymap = getMap();

        mymap[commandname] = x;
    }
};

} // namespace BlockTestCore
