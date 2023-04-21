#include "engine/BasicUtilityFunction.h"
#include <DynamicLoadingUtils.h>
#include <DynamicUtilityFunctionCreator.h>
#include <boost/dll/import.hpp> // for import_alias
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace alica
{

DynamicUtilityFunctionCreator::DynamicUtilityFunctionCreator()
{
    _libraryPath = calculateLibraryPath();
}

std::shared_ptr<BasicUtilityFunction> DynamicUtilityFunctionCreator::createUtility(int64_t utilityFunctionConfId, UtilityFunctionContext& context)
{
    std::string completeLibraryName;
    std::string symbolName =
            context.name + "UtilityFunction"; // symbol to import, append `UtilityFunction` to name because the name is the same as the plan's name
    try {
        completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.libraryName);
    } catch (const std::exception& ex) {
        throw DynamicLoadingException{"utilityFunction", utilityFunctionConfId, symbolName, "", context.libraryName, ex.what()};
    }

    try {
        _utilityFunctionCreator = boost::dll::import_alias<utilityFunctionCreatorType>( // type of imported symbol must be explicitly specified
                completeLibraryName,                                                    // complete path to library also with file name
                symbolName,
                boost::dll::load_mode::append_decorations // do append extensions and prefixes
        );
    } catch (const std::exception& ex) {
        throw DynamicLoadingException{"utilityFunction", utilityFunctionConfId, symbolName, "", context.libraryName, ex.what()};
    }

    std::shared_ptr<BasicUtilityFunction> createdUtilityFunction = _utilityFunctionCreator(context);

    return createdUtilityFunction;
}

} // namespace alica
