#pragma once

#include "engine/logging/Logging.h"

#include <filesystem>
#include <iostream>
#include <sstream>
#include <stack>
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

inline std::vector<std::string> calculateLibraryPath()
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

inline std::string calculateLibraryCompleteName(const std::vector<std::string>& libraryPath, const std::string& libraryName)
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

// Algorithm from internet
inline std::string simplifyPath(const std::string& toSimplify)
{
    std::stack<std::string> myStack;
    std::string dir;
    std::string outPath;

    outPath.append("/");

    int toSimpliftLen = toSimplify.length();
    for (int i = 0; i < toSimpliftLen; i++) {
        dir.clear();
        while (toSimplify[i] == '/')
            i++;

        while (i < toSimpliftLen && toSimplify[i] != '/') {
            dir.push_back(toSimplify[i]);
            i++;
        }

        if (dir.compare("..") == 0) {
            if (!myStack.empty())
                myStack.pop();
        } else if (dir.compare(".") == 0)
            continue;
        else if (dir.length() != 0)
            myStack.push(dir);
    }

    std::stack<std::string> myStackReverted;
    while (!myStack.empty()) {
        myStackReverted.push(myStack.top());
        myStack.pop();
    }

    // Revert and create out string
    while (!myStackReverted.empty()) {
        std::string temp = myStackReverted.top();
        if (myStackReverted.size() != 1)
            outPath.append(temp + "/");
        else
            outPath.append(temp);
        myStackReverted.pop();
    }
    return outPath;
}

} // namespace alica
