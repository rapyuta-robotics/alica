/*
 * ExecutableMetaData.h
 *
 *  Created on: Feb 12, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_PROCESS_MANAGER_SRC_EXECUTABLEMETADATA_H_
#define SUPPLEMENTARY_PROCESS_MANAGER_SRC_EXECUTABLEMETADATA_H_

#include <string>
#include <vector>
#include <map>
#include <sstream>

using namespace std;

namespace supplementary
{

	class ExecutableMetaData
	{
	public:
		ExecutableMetaData(string name, int id, string mode, string execName, string rosPackage, string prefixCmd,string absExecName);
		ExecutableMetaData(string name, int id, string mode, string execName, string rosPackage, string prefixCmd, map<int, vector<char*>> parameterMap, string absExecName);
		virtual ~ExecutableMetaData();

		bool matchSplittedCmdLine(vector<string>& cmdline);
		void addParameterSet(int paramSetId, vector<char*> paramSetValues);

		static const long NOTHING_MANAGED = -1;
		static const int UNKNOWN_PARAMS = -1;
		int id;
		string absExecName;
		string name;
		string execName;
		string rosPackage;
		string prefixCmd;
		map<int, vector<char*>> parameterMap;
		string mode;
	};

} /* namespace supplementary */

inline std::ostream& operator<<(std::ostream &strm, const supplementary::ExecutableMetaData &a)
{
	std::ostringstream resultStream;
	resultStream << "ExecutableMetaData: " << a.name << "(" << a.id << ")\n\tAbsExecName:\t" << a.absExecName << "\n\tMode:\t\t" << a.mode << endl;
	for (auto paramEntry : a.parameterMap)
	{
		resultStream << "\tParamSet " << paramEntry.first << ": ";
		for (char* param : paramEntry.second)
		{
			resultStream << "\t'" << param << "' ";
		}
		resultStream << endl;
	}
	strm  << resultStream.str();
	return strm;
}

#endif /* SUPPLEMENTARY_PROCESS_MANAGER_SRC_EXECUTABLEMETADATA_H_ */
