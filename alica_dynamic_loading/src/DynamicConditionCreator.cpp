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
    std::string completeLibraryName;
    try {
        completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.libraryName);
    } catch (const std::exception& ex) {
        throw DynamicLoadingException{"condition", conditionConfId, context.name, "", context.libraryName, ex.what()};
    }

    try {
        _conditionCreator = boost::dll::import_alias<conditionCreatorType>( // type of imported symbol must be explicitly specified
                completeLibraryName,                                        // complete path to library also with file name
                context.name,                                               // symbol to import
                boost::dll::load_mode::append_decorations                   // do append extensions and prefixes
        );
    } catch (const std::exception& ex) {
        throw DynamicLoadingException{"condition", conditionConfId, context.name, "", context.libraryName, ex.what()};
    }

    std::shared_ptr<BasicCondition> createdCondition = _conditionCreator(context);
    Logging::logDebug("DynamicLoading") << "Loaded condition " << context.name;

    return createdCondition;
}

} // namespace alica
