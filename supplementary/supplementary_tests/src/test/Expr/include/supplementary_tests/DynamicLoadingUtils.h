#pragma once

#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace alica
{

inline std::vector<std::string> tokenizeStr(const std::string& toTokenize, char delimter)
{
    std::vector<std::string> tokens;
    std::stringstream ssToTokenize(toTokenize);
    std::string tmp;

    while (std::getline(ssToTokenize, tmp, delimter)) {
        tokens.push_back(tmp);
    }

    return tokens;
}

inline std::string calculateLibraryPath(const std::string& libraryPathFromConfigFile)
{
    if (libraryPathFromConfigFile != "") {
        std::cerr << "Debug:"
                  << "use library path from Alica.yaml:" << libraryPathFromConfigFile << std::endl;
        return libraryPathFromConfigFile;
    }

    const char* rosPackagePath = std::getenv("ROS_PACKAGE_PATH");
    auto tokens = tokenizeStr(rosPackagePath, ':');
    if (tokens.size() < 1) {
        std::cerr << "Error:"
                  << "Missing ROS_PACKAGE_PATH" << std::endl;
        return "";
    }

    std::cerr << "Debug:"
              << "use library path from ROS_PACKAGE_PATH:" << tokens[0] + "/../lib/" << std::endl;

    return tokens[0] + "/../lib/";
}

inline std::string calculateLibraryCompleteName(const std::string& libraryPath, const std::string& libraryName)
{
    if (libraryName == "") {
        std::cerr << "Error:"
                  << "Empty library name" << std::endl;
        return "";
    }

    return libraryPath + "/lib" + libraryName + ".so";
}

inline bool checkLibraryCompleteName(const std::string& libraryCompleteName, const std::string& entityName)
{
    if (!std::filesystem::exists(libraryCompleteName)) {
        std::cerr << "Error:"
                  << "Lib not exixts in this path:" << libraryCompleteName << std::endl;
        return false;
    } else {
        std::cerr << "Debug:"
                  << "Lib exixts in this path:" << libraryCompleteName << " for:" << entityName << std::endl;
    }
    return true;
}

} // namespace alica
