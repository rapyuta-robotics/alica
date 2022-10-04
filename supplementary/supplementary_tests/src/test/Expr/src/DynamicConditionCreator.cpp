#include "engine/BasicCondition.h"
#include <supplementary_tests/DynamicConditionCreator.h>
#include <supplementary_tests/DynamicLoadingUtils.h>
#include <boost/dll/import.hpp> // for import_alias
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace alica
{

DynamicConditionCreator::~DynamicConditionCreator() {}

std::shared_ptr<BasicCondition> DynamicConditionCreator::createConditions(ConditionContext& context)
{
    std::string libraryPath = calculateLibraryPath(context.libraryPath);
    std::string completeLibraryName = calculateLibraryCompleteName(libraryPath, context.libraryName);
    if (!checkLibraryCompleteName(completeLibraryName, context.name)) {
        return nullptr;
    }

    _conditionCreator = boost::dll::import_alias<conditionCreatorType>( // type of imported symbol must be explicitly specified
            completeLibraryName,                                        // complete path to library also with file name
            context.name,                                               // symbol to import
            boost::dll::load_mode::append_decorations                   // do append extensions and prefixes
    );

    std::shared_ptr<BasicCondition> createdCondition = _conditionCreator(context);

    return createdCondition;
}
} // namespace alica
