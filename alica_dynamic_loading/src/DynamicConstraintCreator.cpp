#include "engine/BasicConstraint.h"
#include <DynamicConstraintCreator.h>
#include <DynamicLoadingUtils.h>
#include <boost/dll/import.hpp> // for import_alias
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace alica
{

DynamicConstraintCreator::DynamicConstraintCreator()
{
    _libraryPath = calculateLibraryPath();
}

std::shared_ptr<BasicConstraint> DynamicConstraintCreator::createConstraint(int64_t constraintConfId, ConstraintContext& context)
{
    std::string completeLibraryName;
    std::string symbolName = context.name + "Constraint"; // symbol to import, append `Constraint` to name because the name is the same as the condition's name
    try {
        completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.libraryName);
    } catch (const DynamicLoadingException& ex) {
        throw DynamicLoadingException{"constraint", constraintConfId, symbolName, "", context.libraryName, ex.what()};
    }

    try {
        _constraintCreator = boost::dll::import_alias<constraintCreatorType>( // type of imported symbol must be explicitly specified
                completeLibraryName,                                          // complete path to library also with file name
                symbolName,
                boost::dll::load_mode::append_decorations // do append extensions and prefixes
        );
    } catch (const std::exception& ex) {
        throw DynamicLoadingException{"constraint", constraintConfId, symbolName, "", context.libraryName, ex.what()};
    }

    std::shared_ptr<BasicConstraint> createdConstraint = _constraintCreator(context);
    Logging::logDebug("DynamicLoading") << "Loaded constraint " << context.name << "Constraint";

    return createdConstraint;
}

} // namespace alica
