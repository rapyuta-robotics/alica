#pragma once

class PathParser
{
public:
    /**
     * Splits the given strings if it finds the given seperator.
     * @param seperator
     * @param path
     * @param ap
     * @return The list of strings after everything was splitted.
     */
    std::vector<std::string> getParams(char seperator, const char* path)
    {
        std::vector<std::string> params = std::vector<std::string>();
        if (path != NULL) {
            const char* temp = path;
            std::string::size_type p = 0;
            std::string::size_type q;
            std::string charString = temp;
            while ((q = charString.find(seperator, p)) != std::string::npos) {
                params.emplace_back(temp, p, q - p);
                p = q + 1;
            }
            params.emplace_back(temp, p, charString.length() - p);
        }
        return params;
    }
};


