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

using namespace std;

namespace supplementary
{

	class ExecutableMetaData
	{
	public:
		ExecutableMetaData(string name, int id, string mode, vector<char*> defaultParams, string absExecName);
		virtual ~ExecutableMetaData();

		int id;
		string absExecName;
		string name;
		vector<char*> defaultParams;
		string mode;

	};

} /* namespace supplementary */

#endif /* SUPPLEMENTARY_PROCESS_MANAGER_SRC_EXECUTABLEMETADATA_H_ */
