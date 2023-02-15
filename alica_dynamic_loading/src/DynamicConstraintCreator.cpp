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
    std::string completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.libraryName);
    if (completeLibraryName.empty()) {
        Logging::logError("DynamicLoading") << "Could not compute the complete library name for creating the constraint: " << context.name;
        return nullptr;
    }

    _constraintCreator = boost::dll::import_alias<constraintCreatorType>( // type of imported symbol must be explicitly specified
            completeLibraryName,                                          // complete path to library also with file name
            context.name + "Constraint",              // symbol to import, append `Constraint` to name because the name is the same as the condition's name
            boost::dll::load_mode::append_decorations // do append extensions and prefixes
    );

    std::shared_ptr<BasicConstraint> createdConstraint = _constraintCreator(context);

    Logging::logDebug("DynamicLoading") << "Loaded constraint " << context.name << "Constraint";

    return createdConstraint;
}

} // namespace alica
