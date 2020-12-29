#include "engine/util/ConfigPathParser.h"

std::vector<std::string> ConfigPathParser::getParams(char separator, const std::string& path)
{
    std::vector<std::string> params = std::vector<std::string>();
    if (!path.empty()) {
        std::string temp = path;
        std::string::size_type p = 0;
        std::string::size_type q;
        std::string charString = temp;
        while ((q = charString.find(separator, p)) != std::string::npos) {
            params.emplace_back(temp, p, q - p);
            p = q + 1;
        }
        params.emplace_back(temp, p, charString.length() - p);
    }
    return params;
}