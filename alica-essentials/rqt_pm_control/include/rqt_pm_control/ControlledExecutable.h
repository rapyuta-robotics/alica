/*
 * ControlledExecutable.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDEXECUTABLE_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDEXECUTABLE_H_

#include <ExecutableMetaData.h>

namespace rqt_pm_control
{

	class ControlledExecutable : public supplementary::ExecutableMetaData
	{
	public:
		ControlledExecutable(string execName, int execId, string mode, vector<char*> defaultParams);
		virtual ~ControlledExecutable();

		string params;
		char state; // The process state (zombie, running, etc)
		unsigned short cpu;
		long int memory;


	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_CONTROLLEDEXECUTABLE_H_ */
