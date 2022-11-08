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

std::vector<std::string> tokenizeStr(const std::string& toTokenize, char delimter);
std::vector<std::string> calculateLibraryPath();
std::string calculateLibraryCompleteName(const std::vector<std::string>& libraryPath, const std::string& libraryName);
std::string simplifyPath(const std::string& toSimplify);

} // namespace alica
