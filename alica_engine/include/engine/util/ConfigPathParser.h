#pragma once

#include <string>
#include <vector>

class ConfigPathParser
{
public:
    /**
     * Splits the given strings if it finds the given separator.
     * @param separator
     * @param path
     * @return The list of strings after everything was splitted.
     */
    std::vector<std::string> getParams(char separator, const std::string& path);
};
