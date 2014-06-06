/*
 * Quantifier.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Quantifier.h"

namespace alica
{

	Quantifier::Quantifier()
	{
		// TODO Auto-generated constructor stub

	}

	Quantifier::~Quantifier()
	{
		// TODO Auto-generated destructor stub
	}

} /* namespace Alica */

 list<string>& alica::Quantifier::getDomainIdentifiers()
{
	return domainIdentifiers;
}

void alica::Quantifier::setDomainIdentifiers(const list<string>& domainIdentifiers)
{
	this->domainIdentifiers = domainIdentifiers;
}
