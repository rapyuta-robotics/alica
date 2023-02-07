#include "engine/BasicPlan.h"
#include "engine/logging/Logging.h"
#include "engine/model/Plan.h"
#include <DynamicLoadingUtils.h>
#include <DynamicPlanCreator.h>
#include <boost/dll/import.hpp> // for import_alias
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace alica
{

DynamicPlanCreator::DynamicPlanCreator()
{
    _libraryPath = calculateLibraryPath();
}

std::unique_ptr<BasicPlan> DynamicPlanCreator::createPlan(int64_t planId, PlanContext& context)
{
    std::string completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.planModel->getLibraryName());
    if (completeLibraryName.empty()) {
        Logging::logError("DynamicLoading") << "Could not compute the complete library name for creating the plan: " << context.name;
        return nullptr;
    }
    _planCreator = boost::dll::import_alias<PlanCreatorType>( // type of imported symbol must be explicitly specified
            completeLibraryName,                              // complete path to library also with file name
            context.planModel->getName(),                     // symbol to import
            boost::dll::load_mode::append_decorations         // do append extensions and prefixes
    );

    std::unique_ptr<BasicPlan> createdPlan = _planCreator(context);

    return createdPlan;
}
} // namespace alica
