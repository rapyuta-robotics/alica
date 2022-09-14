#include "engine/BasicPlan.h"
#include "engine/model/Plan.h"
#include <boost/dll/import.hpp> // for import_alias
#include <boost/filesystem.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <supplementary_tests/DynamicPlanCreator.h>

namespace alica
{

DynamicPlanCreator::DynamicPlanCreator() {}

DynamicPlanCreator::~DynamicPlanCreator() {}

std::unique_ptr<BasicPlan> DynamicPlanCreator::createPlan(int64_t planId, PlanContext& context)
{
    if (context.planModel->getLibraryName() == "") {
        std::cerr << "Error:"
                  << "Empty library name for" << context.planModel->getName() << std::endl;
        return nullptr;
    }

    std::string libraryPath = context.libraryPath + "/lib" + context.planModel->getLibraryName() + ".so";
    if (!boost::filesystem::exists(libraryPath)) {
        std::cerr << "Error:"
                  << "Lib not exixts in this path:" << libraryPath << std::endl;
        return nullptr;
    } else {
        std::cerr << "Debug:"
                  << "Lib exixts in this path:" << libraryPath << std::endl;
    }

    typedef std::unique_ptr<BasicPlan>(PlanCreatorType)(PlanContext&);
    std::function<PlanCreatorType> PlanCreator;
    PlanCreator = boost::dll::import_alias<PlanCreatorType>( // type of imported symbol must be explicitly specified
            libraryPath,                                     // complete path to library also with file name
            context.planModel->getName(),                    // symbol to import
            boost::dll::load_mode::append_decorations        // do append extensions and prefixes
    );

    std::unique_ptr<BasicPlan> createdPlan = PlanCreator(context);

    return createdPlan;
}
} // namespace alica
