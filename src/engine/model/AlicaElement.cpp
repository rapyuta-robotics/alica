/*
 * AlicaElement.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/AlicaElement.h"

namespace alica
{

	AlicaElement::AlicaElement()
	{
		this->id = 0;
	}

	AlicaElement::~AlicaElement()
	{
		// TODO Auto-generated destructor stub
	}

	void AlicaElement::setName(string name)
	{
		this->name = name;
	}
	string AlicaElement::getName() const
	{
		return this->name;
	}
	void AlicaElement::setComment(string comment)
	{
		this->comment = comment;
	}
	string AlicaElement::getComment()
	{
		return this->comment;
	}
	long AlicaElement::getId() const
	{
		return id;
	}
	void AlicaElement::setId(long id)
	{
		this->id = id;
	}
	string AlicaElement::toString() const
	{
		stringstream ss;
		ss << "ID: " << this->getId() << " Name: " << this->name << endl;
		ss << "Comment: " << this->comment << endl;
		return ss.str();
	}

} /* namespace Alica */

