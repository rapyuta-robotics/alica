#include "engine/LibraryLoader.h"

#include <iostream>

#include <boost/dll/import.hpp>
#include <boost/dll/shared_library.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/filesystem.hpp>
#include <boost/version.hpp>

#if BOOST_VERSION >= 107600 // 1.76.0
#define boost_dll_import boost::dll::import_symbol
#else
#define boost_dll_import boost::dll::import
#endif

namespace alica
{

LibraryLoader::LibraryLoader() {}

bool LibraryLoader::load(const std::string& name, const std::string& companyname, const std::string& additionalPath)
{
    std::string currentPath;
    std::string extension;

    std::cout << "Info:"
              << "LibraryLoader load" << std::endl;

    std::string libraryPath{additionalPath + "/" + companyname + "/" + name + ".so"};

    if (!boost::filesystem::exists(libraryPath)) {
        std::cout << "Error:"
                  << "Try load lib:" << libraryPath << std::endl;
        return false;
    }

    std::cout << "Info:"
              << "Success load lib:" << libraryPath << std::endl;
    try {
        boost::function<funcptr> startFunction = boost_dll_import<funcptr>(libraryPath, "Stop", boost::dll::load_mode::rtld_lazy);
        boost::function<funcptr> stopFunction = boost_dll_import<funcptr>(libraryPath, "Stop", boost::dll::load_mode::rtld_lazy);
        boost::function<funcptr1> configureFunction = boost_dll_import<funcptr1>(libraryPath, "Configure", boost::dll::load_mode::rtld_lazy);

        stopFunction_.emplace_back(stopFunction);

       
    } catch (boost::exception const& e) {
        std::cout << "------------------" << boost::diagnostic_information(e, true) << std::endl;
        std::cout << "Error:"
                  << "Lib:" << libraryPath << " error missing Configure/Stop/Start function in lib----" << boost::diagnostic_information(e, true) << std::endl;
        return false;
    } catch (std::exception& e) {
        std::string error;
        error = e.what();
    } catch (...) {
 std::cout << "Error:"
                  << "Lib:" << libraryPath << "Unknown error" << std::endl;
    }
    std::cout << "Info:"
              << "Load lib ok:" << libraryPath << std::endl;

    return true;
}

LibraryLoader::~LibraryLoader() {}

void LibraryLoader::stop()
{
    /*
    TXLOG(Severity::debug) << "LibraryLoader stop" << std::endl;

    std::for_each(stopFunction_.rbegin(), stopFunction_.rend(), [](const auto current) {
        if (current)
            current(nullptr, nullptr);
    });*/
}

} // namespace alica
