/*
 * RobotRoleUtility.h
 *
 *  Created on: Jul 1, 2014
 *      Author: Stefan Jakob
 */

#ifndef ROBOTROLEUTILITY_H_
#define ROBOTROLEUTILITY_H_

namespace alica
{


	class RobotProperties;
	class Role;

	class RobotRoleUtility
	{
		public:
			RobotRoleUtility(double dUtilityValue, RobotProperties* robot, Role* role);
			virtual ~RobotRoleUtility();
			static bool compareTo(RobotRoleUtility* thisOne, RobotRoleUtility* otherOne);
			RobotProperties* getRobot();
			Role* getRole();
			double getUtilityValue();

		protected:
			RobotProperties* robot;
			Role* role;
			double utilityValue;


	};

} /* namespace alica */

#endif /* ROBOTROLEUTILITY_H_ */
