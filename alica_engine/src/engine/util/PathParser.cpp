#include "engine/util/PathParser.h"

std::vector<std::string> PathParser::getParams(char seperator, const char* path)
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