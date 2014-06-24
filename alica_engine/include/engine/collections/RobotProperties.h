/*
 * RobotProperties.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#ifndef ROBOTPROPERTIES_H_
#define ROBOTPROPERTIES_H_

using namespace std;

#include <string>

namespace alica
{

	class RobotProperties
	{
	public:
		RobotProperties();
		RobotProperties(string name);
		virtual ~RobotProperties();
		int getId() const;
		void setId(int id);
		const string& getName() const;
		void setName(const string& name);

	protected:
		int id = -1;
		string name;
	};

} /* namespace alica */

#endif /* ROBOTPROPERTIES_H_ */
