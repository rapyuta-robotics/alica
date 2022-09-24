#include "engine/BasicPlan.h"
#include "engine/model/Plan.h"
#include <alica/DynamicPlanCreator.h>
#include <boost/dll/import.hpp> // for import_alias
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace alica
{

DynamicPlanCreator::DynamicPlanCreator(const std::string& defaultLibraryPath)
        : _currentLibraryPath(defaultLibraryPath + _libraryRelativePath)
{
}

DynamicPlanCreator::~DynamicPlanCreator() {}

std::unique_ptr<BasicPlan> DynamicPlanCreator::createPlan(int64_t planId, PlanContext& context)
{
    if (context.libraryPath != "") {
        _currentLibraryPath = context.libraryPath;
        std::cerr << "Debug:"
                  << "use library path from Alica.yaml:" << _currentLibraryPath << std::endl;
    } else {
        std::cerr << "Debug:"
                  << "library path:" << _currentLibraryPath << std::endl;
    }

    if (context.planModel->getLibraryName() == "") {
        std::cerr << "Error:"
                  << "Empty library name for" << context.planModel->getName() << std::endl;
        return nullptr;
    }

    std::string libraryPath = _currentLibraryPath + "/lib" + context.planModel->getLibraryName() + ".so";
    if (!std::filesystem::exists(libraryPath)) {
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
