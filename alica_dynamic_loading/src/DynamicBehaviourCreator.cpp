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
    std::string completeLibraryName;
    try {
        completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.behaviourModel->getLibraryName());
    } catch (const std::exception& ex) {
        throw DynamicLoadingException{
                "behaviour", behaviourId, context.name, context.behaviourModel->getImplementationName(), context.behaviourModel->getLibraryName(), ex.what()};
    }

    try {
        _behaviourCreator = boost::dll::import_alias<behaviourCreatorType>( // type of imported symbol must be explicitly specified
                completeLibraryName,                                        // complete path to library also with file name
                context.behaviourModel->getImplementationName(),            // symbol to import
                boost::dll::load_mode::append_decorations                   // do append extensions and prefixes
        );
    } catch (const std::exception& ex) {
        // import_alias can throw boost::system::system_error or std::bad_alloc, this block handles both since each of them derives from std::exception
        throw DynamicLoadingException{
                "behaviour", behaviourId, context.name, context.behaviourModel->getImplementationName(), context.behaviourModel->getLibraryName(), ex.what()};
    }

    std::unique_ptr<BasicBehaviour> createdBehaviour = _behaviourCreator(context);
    Logging::logDebug("DynamicLoading") << "Loaded behavior " << context.behaviourModel->getName();

    return createdBehaviour;
}
} // namespace alica
