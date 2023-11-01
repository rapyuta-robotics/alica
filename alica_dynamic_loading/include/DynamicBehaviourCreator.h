#pragma once
#include <engine/IBehaviourCreator.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace alica
{

class BasicBehaviour;

class DynamicBehaviourCreator : public IBehaviourCreator
{
public:
    DynamicBehaviourCreator();
    virtual ~DynamicBehaviourCreator(){};
    std::unique_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, BehaviourContext& context) override;

private:
    typedef std::unique_ptr<BasicBehaviour>(behaviourCreatorType)(BehaviourContext&);
    /*
     * Each symbol is stored in the map for 2 reasons:
     * 1. The dll is unloaded when the last symbol from that dll is destructed so storing the symbols here is
     * necessary so that the behaviour is not accessed after the dll is unloaded
     * 2. The symbol is only loaded once (consequently the dll is also loaded only once) potentially improving performance
     * by reducing repeated loads & unloads of the dll
     */
    std::unordered_map<std::string, std::function<behaviourCreatorType>> _behaviourCreatorMap;
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
