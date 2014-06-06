/*
 * Variable.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Variable.h"

namespace alica
{

	Variable::Variable()
	{
		// TODO Auto-generated constructor stub

	}

	Variable::~Variable()
	{
		// TODO Auto-generated destructor stub
	}

	Variable::Variable(long id, string name, string type) :
			Variable()
	{
		this->id = id;
		this->name = name;
		this->type = type;
	}

	string Variable::toString()
	{
		stringstream ss;
		ss << "[Variable: Name=" << name << " Id=" << id << endl;
		return ss.str();
	}

//============= Getter and Setter ======================

	const string& Variable::getType() const
	{
		return type;
	}

	void Variable::setType(const string& type)
	{
		this->type = type;
	}
} /* namespace Alica */

