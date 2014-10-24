/*
 * Parameters.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: Stefan Jakob
 */

#include <engine/model/Parameter.h>

namespace alica
{

	Parameter::Parameter()
	{
		this->key = "";
		this->value = "";
	}

	Parameter::~Parameter()
	{
	}

	string Parameter::getKey()
	{
		return key;
	}

	void Parameter::setKey(string key)
	{
		this->key = key;
	}

	string Parameter::getValue()
	{
		return value;
	}

	void Parameter::setValue(string value)
	{
		this->value = value;
	}

} /* namespace alica */
