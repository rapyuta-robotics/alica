#include "engine/BasicBehaviour.h"
#include <boost/dll/import.hpp> // for import_alias
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <supplementary_tests/DynamicBehaviourCreator.h>
#include <supplementary_tests/DynamicLoadingUtils.h>
#include <vector>

namespace alica
{

DynamicBehaviourCreator::DynamicBehaviourCreator() {}

DynamicBehaviourCreator::~DynamicBehaviourCreator() {}

std::unique_ptr<BasicBehaviour> DynamicBehaviourCreator::createBehaviour(int64_t behaviourId, BehaviourContext& context)
{
    std::string libraryPath = calculateLibraryPath();
    std::string completeLibraryName = calculateLibraryCompleteName(libraryPath, context.behaviourModel->getLibraryName());
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
