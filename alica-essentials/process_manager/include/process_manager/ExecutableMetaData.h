#pragma once

#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace essentials
{

class ExecutableMetaData
{
public:
    ExecutableMetaData(
            std::string name, int id, std::string mode, std::string execName, std::string rosPackage, std::string prefixCmd, std::string absExecName);
    ExecutableMetaData(std::string name, int id, std::string mode, std::string execName, std::string rosPackage, std::string prefixCmd,
            std::map<int, std::vector<char*>> parameterMap, std::string absExecName);
    virtual ~ExecutableMetaData();

    bool matchSplittedCmdLine(std::vector<std::string>& cmdline);
    void addParameterSet(int paramSetId, std::vector<char*> paramSetValues);

    static const long NOTHING_MANAGED = -1;
    static const int UNKNOWN_PARAMS = -1;
    int id;
    std::string absExecName;
    std::string name;
    std::string execName;
    std::string rosPackage;
    std::string prefixCmd;
    std::map<int, std::vector<char*>> parameterMap;
    std::string mode;
};

} /* namespace  essentials */

inline std::ostream& operator<<(std::ostream& strm, const essentials::ExecutableMetaData& a)
{
    std::ostringstream resultStream;
    resultStream << "ExecutableMetaData: " << a.name << "(" << a.id << ")\n\tAbsExecName:\t" << a.absExecName << "\n\tMode:\t\t" << a.mode << std::endl;
    for (auto paramEntry : a.parameterMap) {
        resultStream << "\tParamSet " << paramEntry.first << ": ";
        for (char* param : paramEntry.second) {
            resultStream << "\t'" << param << "' ";
        }
        resultStream << std::endl;
    }
    strm << resultStream.str();
    return strm;
}
