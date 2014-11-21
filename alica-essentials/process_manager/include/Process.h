/*
 * Process.h
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#ifndef PROCESS_H_
#define PROCESS_H_

#define PROC_DEBUG

#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

using namespace std;

namespace supplementary
{

	class Process
	{
	public:
		Process(long pid, string params);
		Process(long pid);
		virtual ~Process();
		void update();
		string getRobot();
		string toString();
	private:
		long pid;
		char state;
		string robot;
		string params;
	};

} /* namespace supplementary */

#endif /* PROCESS_H_ */

