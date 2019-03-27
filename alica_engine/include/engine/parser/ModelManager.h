#pragma once

#include <string>

namespace alica
{

/**
 * The default parser, parsing the YAML encoding of an ALICA plan-tree
 */
class Parser
{
public:
    void parseFile(const std::string& currentFile, const std::string& type);
};
} // namespace alica
