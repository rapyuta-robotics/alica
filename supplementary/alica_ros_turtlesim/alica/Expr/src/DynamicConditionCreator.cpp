#include "engine/BasicCondition.h"
#include <alica/DynamicConditionCreator.h>
#include <boost/dll/import.hpp> // for import_alias
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace alica
{

DynamicConditionCreator::DynamicConditionCreator(const std::string& defaultLibraryPath)
        : _defaultLibraryPath(defaultLibraryPath + _libraryRelativePath)
{
}

DynamicConditionCreator::~DynamicConditionCreator() {}

std::shared_ptr<BasicCondition> DynamicConditionCreator::createConditions(ConditionContext& context)
{
    if (context.libraryPath != "") {
        _defaultLibraryPath = context.libraryPath;
        std::cerr << "Debug:"
                  << "use library path from Alica.yaml:" << _defaultLibraryPath << std::endl;
    } else {
        std::cerr << "Debug:"
                  << "library path:" << _defaultLibraryPath << std::endl;
    }

    if (context.libraryName == "") {
        std::cerr << "Error:"
                  << "Empty library name for" << context.name << std::endl;
        return nullptr;
    }

    std::string libraryPath = _defaultLibraryPath + "/lib" + context.libraryName + ".so";
    if (!std::filesystem::exists(libraryPath)) {
        std::cerr << "Error:"
                  << "Lib not exixts in this path:" << libraryPath << std::endl;
        return nullptr;
    } else {
        std::cerr << "Debug:"
                  << "Lib exixts in this path:" << libraryPath << " for:" << context.name << std::endl;
    }

    typedef std::unique_ptr<BasicCondition>(conditionCreatorType)(ConditionContext&);
    std::function<conditionCreatorType> conditionCreator;
    conditionCreator = boost::dll::import_alias<conditionCreatorType>( // type of imported symbol must be explicitly specified
            libraryPath,                                               // complete path to library also with file name
            context.name,                                              // symbol to import
            boost::dll::load_mode::append_decorations                  // do append extensions and prefixes
    );

    std::unique_ptr<BasicCondition> createdCondition = conditionCreator(context);

    return createdCondition;
}
} // namespace alica
