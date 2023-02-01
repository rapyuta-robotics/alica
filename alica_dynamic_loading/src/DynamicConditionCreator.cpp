#include "engine/BasicCondition.h"
#include <DynamicConditionCreator.h>
#include <DynamicLoadingUtils.h>
#include <boost/dll/import.hpp> // for import_alias
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace alica
{

DynamicConditionCreator::DynamicConditionCreator()
{
    _libraryPath = calculateLibraryPath();
}

std::shared_ptr<BasicCondition> DynamicConditionCreator::createConditions(int64_t conditionConfId, ConditionContext& context)
{
    std::string completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.libraryName);
    if (completeLibraryName.empty()) {
        Logging::logError("DynamicLoading") << "Could not compute the complete library name for creating the condition: " << context.name;
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
