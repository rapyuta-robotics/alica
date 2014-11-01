/*
 * ROSProcess.h
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROSPROCESS_H_
#define ROSPROCESS_H_

#include <string>

using namespace std;

namespace supplementary
{

	class ROSProcess
	{
	public:
		ROSProcess(short id, string rospkg, string rosexecutable, string params);
		virtual ~ROSProcess();
		int getId() const;
		void setId(int id);
		const string& getParams() const;
		void setParams(const string& params);
		const string& getRosexecutable() const;
		void setRosexecutable(const string& rosexecutable);
		const string& getRospkg() const;
		void setRospkg(const string& rospkg);

	private:
		short id;
		string rospkg;
		string rosexecutable;
		string params;
	};

} /* namespace supplementary */

#endif /* ROSPROCESS_H_ */
