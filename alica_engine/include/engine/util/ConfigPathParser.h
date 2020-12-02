#pragma once

#include <vector>
#include <string>

class ConfigPathParser
{
public:
    /**
     * Splits the given strings if it finds the given seperator.
     * @param seperator
     * @param path
     * @param ap
     * @return The list of strings after everything was splitted.
     */
    std::vector<std::string> getParams(char seperator, const char* path);
};


