/*
 * RobotMetaData.h
 *
 *  Created on: Feb 12, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_PROCESS_MANAGER_SRC_ROBOTMETADATA_H_
#define SUPPLEMENTARY_PROCESS_MANAGER_SRC_ROBOTMETADATA_H_

#include <string>

using namespace std;

namespace supplementary
{

	class RobotMetaData
	{
	public:
		RobotMetaData(string name, int id);
		virtual ~RobotMetaData();

		int id;
		string name;
	};

} /* namespace supplementary */

#endif /* SUPPLEMENTARY_PROCESS_MANAGER_SRC_ROBOTMETADATA_H_ */
