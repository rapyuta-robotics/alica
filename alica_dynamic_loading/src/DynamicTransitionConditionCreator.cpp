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

TransitionConditionCallback DynamicTransitionConditionCreator::createConditions(int64_t conditionConfId, TransitionConditionContext& context)
{
    std::string completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.libraryName);
    if (completeLibraryName.empty())
        return nullptr;
    auto fun = boost::dll::import_alias<transitionConditionFunctionType>( // type of imported symbol must be explicitly specified
            completeLibraryName,                                          // complete path to library also with file name
            context.name,                                                 // symbol to import
            boost::dll::load_mode::append_decorations                     // do append extensions and prefixes
    );
    return fun;
}

} // namespace alica
