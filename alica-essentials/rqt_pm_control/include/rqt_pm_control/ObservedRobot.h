/*
 * ObservedRobot.h
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_OBSERVEDROBOT_H_
#define SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_OBSERVEDROBOT_H_

#include <RobotMetaData.h>

namespace rqt_pm_control
{

	class ObservedRobot : public supplementary::RobotMetaData
	{
	public:
		ObservedRobot(string robotName, int robotId);
		virtual ~ObservedRobot();
	};

} /* namespace rqt_pm_control */

#endif /* SUPPLEMENTARY_RQT_PM_CONTROL_SRC_RQT_PM_CONTROL_OBSERVEDROBOT_H_ */
