#include "engine/BasicBehaviour.h"
#include <boost/dll/import.hpp> // for import_alias
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <alica/DynamicBehaviourCreator.h>

namespace alica
{

DynamicBehaviourCreator::DynamicBehaviourCreator() {}

DynamicBehaviourCreator::~DynamicBehaviourCreator() {}

std::unique_ptr<BasicBehaviour> DynamicBehaviourCreator::createBehaviour(int64_t behaviourId, BehaviourContext& context)
{
    if (context.behaviourModel->getLibraryName() == "") {
        std::cerr << "Error:"
                  << "Empty library name for" << context.behaviourModel->getName() << std::endl;
        return nullptr;
    }

    std::string libraryPath = context.libraryPath + "/lib" + context.behaviourModel->getLibraryName() + ".so";
    if (!boost::filesystem::exists(libraryPath)) {
        std::cerr << "Error:"
                  << "Lib not exixts in this path:" << libraryPath << std::endl;
        return nullptr;
    } else {
        std::cerr << "Debug:"
                  << "Lib exixts in this path:" << libraryPath << std::endl;
    }

    typedef std::unique_ptr<BasicBehaviour>(behaviourCreatorType)(BehaviourContext&);
    std::function<behaviourCreatorType> behaviourCreator;
    behaviourCreator = boost::dll::import_alias<behaviourCreatorType>( // type of imported symbol must be explicitly specified
            libraryPath,                                               // complete path to library also with file name
            context.behaviourModel->getName(),                         // symbol to import
            boost::dll::load_mode::append_decorations                  // do append extensions and prefixes
    );

    std::unique_ptr<BasicBehaviour> createdBehaviour = behaviourCreator(context);

    return createdBehaviour;
}
} // namespace alica
