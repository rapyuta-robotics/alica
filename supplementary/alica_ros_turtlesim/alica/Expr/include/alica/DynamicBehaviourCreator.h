#pragma once
#include <engine/IBehaviourCreator.h>

#include <memory>

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
};

} /* namespace alica */
