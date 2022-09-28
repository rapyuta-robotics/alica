#pragma once
#include <engine/IBehaviourCreator.h>

#include <memory>
#include <functional>

namespace alica
{

class BasicBehaviour;

class DynamicBehaviourCreator : public IBehaviourCreator
{
public:
    DynamicBehaviourCreator(const std::string& defaultLibraryPath);
    virtual ~DynamicBehaviourCreator();
    std::unique_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, BehaviourContext& context) override;

private:
    const std::string _libraryRelativePath{"/../../../lib/"};
    std::string _currentLibraryPath;

    typedef std::unique_ptr<BasicBehaviour>(behaviourCreatorType)(BehaviourContext&);
    std::function<behaviourCreatorType> _behaviourCreator;
};

} /* namespace alica */
