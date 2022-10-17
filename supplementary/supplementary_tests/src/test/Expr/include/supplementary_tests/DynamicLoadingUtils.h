#pragma once

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

inline std::string calculateLibraryPath()
{
    const char* ldLibraryPath = std::getenv("LD_LIBRARY_PATH");
    auto tokens = tokenizeStr(ldLibraryPath, ':');
    if (tokens.size() < 1) {
        std::cerr << "Error:"
                  << "Missing LD_LIBRARY_PATH" << std::endl;
        return "";
    }

    std::cerr << "Debug:"
              << "use library path from LD_LIBRARY_PATH:" << tokens[0] << std::endl;

    return tokens[0];
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
