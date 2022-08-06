#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "engine/Types.h"

namespace alica
{

class BehaviourRegister
{
public:
    static CreationFuncDepot& getMap()
    {
        static CreationFuncDepot map_;
        return map_;
    }

public:
    static CreationFunction getCreatorFunction(const std::string& commandname)
    {
        CreationFunction func;
        CreationFuncDepot& mymap = getMap();
        if (mymap.find(commandname) != mymap.end())
            func = mymap[commandname];
        else
            std::cerr << "Error"
                      << "Function creator not found for:" << commandname << std::endl;
        return func;
    }

    void Dump()
    {
        std::cerr << "Info"
                  << "--------" << std::endl;
        CreationFuncDepot& mymap = getMap();
        for (auto current : mymap) {
            std::cerr << "Info"
                      << "--->" << current.first << std::endl;
        }
        std::cerr << "Info"
                  << "--------" << std::endl;
    }

    virtual ~BehaviourRegister()
    {
        CreationFuncDepot& mymap = getMap();
        mymap.clear();
    };
};

template <typename T>
class DerivedBehaviourRegister : public BehaviourRegister
{
public:
    explicit DerivedBehaviourRegister(const std::string& commandname)
    {
        CreationFunction x = [](BehaviourContext& context) { return std::make_unique<T>(context); };
        CreationFuncDepot& mymap = getMap();

        mymap[commandname] = x;
    }
};
}; // namespace alica