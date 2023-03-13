#include "engine/BasicBehaviour.h"
#include <DynamicBehaviourCreator.h>
#include <DynamicLoadingUtils.h>
#include <boost/dll/import.hpp> // for import_alias
#include <cstdlib>
#include <engine/logging/Logging.h>
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
    if (completeLibraryName.empty()) {
        Logging::logError("DynamicLoading") << "Could not compute the complete library name for creating the behaviour: " << context.name;
        return nullptr;
    }

    _behaviourCreator = boost::dll::import_alias<behaviourCreatorType>( // type of imported symbol must be explicitly specified
            completeLibraryName,                                        // complete path to library also with file name
            context.behaviourModel->getImplementationName(),            // symbol to import
            boost::dll::load_mode::append_decorations                   // do append extensions and prefixes
    );
    Logging::logDebug("DynamicLoading") << "Loaded behavior " << context.behaviourModel->getName();

    std::unique_ptr<BasicBehaviour> createdBehaviour = _behaviourCreator(context);

    return createdBehaviour;
}
} // namespace alica
