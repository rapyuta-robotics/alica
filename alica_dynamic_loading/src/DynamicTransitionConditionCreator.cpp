#include "engine/model/Transition.h"
#include <DynamicLoadingUtils.h>
#include <DynamicTransitionConditionCreator.h>
#include <boost/dll/import.hpp> // for import_alias
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace alica
{

DynamicTransitionConditionCreator::DynamicTransitionConditionCreator()
{
    _libraryPath = calculateLibraryPath();
}

TransitionConditionCallback DynamicTransitionConditionCreator::createConditions(int64_t conditionId, TransitionConditionContext& context)
{
    std::string completeLibraryName;
    try {
        completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.libraryName);
    } catch (const std::exception& ex) {
        throw DynamicLoadingException{"transitionCondition", conditionId, context.name, "", context.libraryName, ex.what()};
    }

    try {
        auto fun = boost::dll::import_alias<transitionConditionFunctionType>( // type of imported symbol must be explicitly specified
                completeLibraryName,                                          // complete path to library also with file name
                context.name,                                                 // symbol to import
                boost::dll::load_mode::append_decorations                     // do append extensions and prefixes
        );
        Logging::logDebug("DynamicLoading") << "Loaded transition condition " << context.name;
        return fun;
    } catch (const std::exception& ex) {
        throw DynamicLoadingException{"transitionCondition", conditionId, context.name, "", context.libraryName, ex.what()};
    }
}

} // namespace alica
