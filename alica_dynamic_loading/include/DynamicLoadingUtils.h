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

struct DynamicLoadingException : std::runtime_error
{
    DynamicLoadingException(const std::string& errorMsg);
    DynamicLoadingException(const std::string& element, int64_t id, const std::string& name, const std::string& implName, const std::string& libName,
            const std::string& errorMsg);
};

std::vector<std::string> tokenizeStr(const std::string& toTokenize, char delimter);
std::vector<std::string> calculateLibraryPath();
std::string calculateLibraryCompleteName(const std::vector<std::string>& libraryPath, const std::string& libraryName);

} // namespace alica
