/*
 * RobotRoleUtility.h
 *
 *  Created on: Jul 1, 2014
 *      Author: Stefan Jakob
 */

#ifndef ROBOTROLEUTILITY_H_
#define ROBOTROLEUTILITY_H_

#include <memory>

using namespace std;

namespace alica
{


	class RobotProperties;
	class Role;

	class RobotRoleUtility
	{
		public:
			RobotRoleUtility(double dUtilityValue, shared_ptr<RobotProperties> robot, Role* role);
			virtual ~RobotRoleUtility();
			static bool compareTo(RobotRoleUtility* thisOne, RobotRoleUtility* otherOne);
			shared_ptr<RobotProperties> getRobot();
			Role* getRole();
			double getUtilityValue();

		protected:
			shared_ptr<RobotProperties> robot;
			Role* role;
			double utilityValue;


	};

} /* namespace alica */

#endif /* ROBOTROLEUTILITY_H_ */
