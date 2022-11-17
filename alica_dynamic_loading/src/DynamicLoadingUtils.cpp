#include "DynamicLoadingUtils.h"
#include "engine/logging/Logging.h"

namespace alica
{

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
        Logging::logError("DL") << "Error:"
                                << "Missing LD_LIBRARY_PATH variable";
        return std::vector<std::string>();
    }

    auto tokens = tokenizeStr(ldLibraryPath, ':');
    if (tokens.empty()) {
        Logging::logError("DL") << "Error:"
                                << "Missing LD_LIBRARY_PATH";
        return std::vector<std::string>();
    }

    return tokens;
}

std::string calculateLibraryCompleteName(const std::vector<std::string>& libraryPath, const std::string& libraryName)
{
    if (libraryName == "") {
        Logging::logError("DL") << "Error:"
                                << "Empty library name";
        return "";
    }

    for (const std::string& current : libraryPath) {
        std::string completeName = current + "/lib" + libraryName + ".so";
        if (std::filesystem::exists(completeName)) {
            Logging::logDebug("DL") << "Debug:"
                                    << "Lib exixts in this path:" << completeName;
            return completeName;
        }
    }
    Logging::logError("DL") << "Error:"
                            << "Lib not exixts in LD_CONFIG_PATH library name:" << libraryName;
    return "";
}

} // namespace alica
