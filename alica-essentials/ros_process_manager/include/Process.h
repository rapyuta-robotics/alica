/*
 * Process.h
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#ifndef PROCESS_H_
#define PROCESS_H_

#include <string>
#include <vector>

using namespace std;

namespace supplementary
{

	class Process
	{
	public:
		Process(short id, string rospkg, string rosexecutable, string params);
		virtual ~Process();
		int getId() const;
		void setId(int id);
		const string& getParams() const;
		void setParams(const string& params);
		const string& getExecutable() const;
		void setExecutable(const string& rosexecutable);

	private:
		short id;
		string executable;
		string params;
		vector<int> pids;
	};

} /* namespace supplementary */

#endif /* PROCESS_H_ */
