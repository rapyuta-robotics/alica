#include "essentials/ConsoleCommandHelper.h"
#include <stdio.h>

using namespace std;

namespace essentials
{

ConsoleCommandHelper::ConsoleCommandHelper()
{
    // TODO Auto-generated constructor stub
}

ConsoleCommandHelper::~ConsoleCommandHelper()
{
    // TODO Auto-generated destructor stub
}

std::string ConsoleCommandHelper::exec(const char* cmd)
{
    FILE* pipe = popen(cmd, "r");
    if (!pipe)
        return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

} /* namespace essentials */
