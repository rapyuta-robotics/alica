#include "engine/BasicBehaviour.h"
#include <DynamicBehaviourCreator.h>
#include <DynamicLoadingUtils.h>
#include <boost/dll/import.hpp> // for import_alias
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace alica
{

DynamicBehaviourCreator::DynamicBehaviourCreator()
{
    _libraryPath = calculateLibraryPath();
}

std::unique_ptr<BasicBehaviour> DynamicBehaviourCreator::createBehaviour(int64_t behaviourId, BehaviourContext& context)
{
    std::string completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.behaviourModel->getLibraryName());
    if (!checkLibraryCompleteName(completeLibraryName, context.behaviourModel->getName())) {
        return nullptr;
    }

    _behaviourCreator = boost::dll::import_alias<behaviourCreatorType>( // type of imported symbol must be explicitly specified
            completeLibraryName,                                        // complete path to library also with file name
            context.behaviourModel->getName(),                          // symbol to import
            boost::dll::load_mode::append_decorations                   // do append extensions and prefixes
    );

    std::unique_ptr<BasicBehaviour> createdBehaviour = _behaviourCreator(context);

    return createdBehaviour;
}
} // namespace alica
