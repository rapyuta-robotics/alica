#include "DynamicLoadingUtils.h"
#include "engine/logging/Logging.h"

namespace alica
{

DynamicLoadingException::DynamicLoadingException(const std::string& errorMsg)
        : std::runtime_error(errorMsg)
{
}
DynamicLoadingException::DynamicLoadingException(
        const std::string& element, int64_t id, const std::string& name, const std::string& implName, const std::string& libName, const std::string& errorMsg)
        : std::runtime_error(
                  stringify("Could not load ", element, ": ", name, ", id: ", id, ", implName: ", implName, " from library: ", libName, ", reason: ", errorMsg))
{
}

std::vector<std::string> tokenizeStr(const std::string& toTokenize, char delimter)
{
    std::vector<std::string> tokens;
    std::stringstream ssToTokenize(toTokenize);
    std::string tmp;

    while (std::getline(ssToTokenize, tmp, delimter)) {
        tokens.push_back(tmp);
    }

    return tokens;
}

std::vector<std::string> calculateLibraryPath()
{
    const char* ldLibraryPath = std::getenv("LD_LIBRARY_PATH");
    if (!ldLibraryPath) {
        throw DynamicLoadingException{"could not load LD_LIBRARY_PATH"};
    }

    auto tokens = tokenizeStr(ldLibraryPath, ':');
    if (tokens.empty()) {
        throw DynamicLoadingException{"LD_LIBRARY_PATH is empty"};
    }

    return tokens;
}

std::string calculateLibraryCompleteName(const std::vector<std::string>& libraryPath, const std::string& libraryName)
{
    if (libraryName.empty()) {
        throw DynamicLoadingException{"library name is empty"};
    }

    for (const std::string& current : libraryPath) {
        std::string completeName = current + "/lib" + libraryName + ".so";
        if (std::filesystem::exists(completeName)) {
            return completeName;
        }
    }
    throw DynamicLoadingException{"could not find library: lib" + libraryName + ".so, in paths defined in LD_LIBRARY_PATH"};
}

} // namespace alica
