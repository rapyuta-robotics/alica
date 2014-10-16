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
#include <map>
#include <sstream>
#include <SystemConfig.h>
#include <algorithm>
#include <memory>

namespace alica
{

	class Characteristic;
	class Capability;
	class AlicaEngine;

	class RobotProperties
	{
	public:
		RobotProperties();
		RobotProperties(AlicaEngine* ae, string name);
		virtual ~RobotProperties();
		int getId() const;
		void setId(int id);
		const string& getName() const;
		void setName(const string& name);
		map<string, Characteristic*>& getCharacteristics();
		const string& getDefaultRole() const;
		void setDefaultRole(const string& defaultRole);
		string toString();

	protected:
		int id = -1;
		string name;
		string defaultRole;
		map<string, Characteristic*> characteristics;
		map<long, Capability*> capabilities;

	};

} /* namespace alica */

#endif /* ROBOTPROPERTIES_H_ */
