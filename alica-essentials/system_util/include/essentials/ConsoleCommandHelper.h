#pragma once 

#include <string>

namespace essentials
{

class ConsoleCommandHelper
{
public:
    ConsoleCommandHelper();
    virtual ~ConsoleCommandHelper();
    static std::string exec(const char* cmd);
};

} /* namespace essentials */

