/*
 * RobotProperties.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#include <engine/collections/RobotProperties.h>

namespace alica
{

	RobotProperties::RobotProperties()
	{
		 //TODO Auto-generated constructor stub

	}

	RobotProperties::~RobotProperties()
	{
		// TODO Auto-generated destructor stub
	}

	int RobotProperties::getId() const
	{
		return id;
	}

	RobotProperties::RobotProperties(string name)
	{
		this->name = name;
	}

	void RobotProperties::setId(int id)
	{
		this->id = id;
	}

	const string& RobotProperties::getName() const
	{
		return name;
	}

	void RobotProperties::setName(const string& name)
	{
		this->name = name;
	}

} /* namespace alica */
