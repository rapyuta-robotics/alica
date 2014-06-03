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
		// TODO Auto-generated constructor stub

	}

	AlicaElement::~AlicaElement()
	{
		// TODO Auto-generated destructor stub
	}

	void AlicaElement::setName(string name){
		this->name = name;
	}
	string AlicaElement::getName(){
		return this->name;
	}
	void AlicaElement::setComment(string comment){
		this->comment = comment;
	}
	string AlicaElement::getComment(){
		return this->comment;
	}

} /* namespace Alica */
