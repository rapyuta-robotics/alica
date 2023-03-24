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
    std::string completeLibraryName;
    try {
        completeLibraryName = calculateLibraryCompleteName(_libraryPath, context.planModel->getLibraryName());
    } catch (const DynamicLoadingException& ex) {
        throw DynamicLoadingException{"plan", planId, context.name, context.planModel->getImplementationName(), context.planModel->getLibraryName(), ex.what()};
    }

    try {
        _planCreator = boost::dll::import_alias<PlanCreatorType>( // type of imported symbol must be explicitly specified
                completeLibraryName,                              // complete path to library also with file name
                context.planModel->getImplementationName(),       // symbol to import
                boost::dll::load_mode::append_decorations         // do append extensions and prefixes
        );
    } catch (const std::exception& ex) {
        throw DynamicLoadingException{"plan", planId, context.name, context.planModel->getImplementationName(), context.planModel->getLibraryName(), ex.what()};
    }

    std::unique_ptr<BasicPlan> createdPlan = _planCreator(context);

    // TODO: Reenable if we can avoid creating running plans every time we do an assignment evaluation
    // Logging::logDebug("DynamicLoading") << "Loaded plan " << context.planModel->getName();

    return createdPlan;
}
} // namespace alica
