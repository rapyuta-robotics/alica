/*
 * Quantifier.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef QUANTIFIER_H_
#define QUANTIFIER_H_

using namespace std;

#include "AlicaElement.h"
#include <list>
#include <string>




namespace alica
{

	class Quantifier : public AlicaElement
	{
	public:
		Quantifier();
		virtual ~Quantifier();
		list<string>& getDomainIdentifiers() ;
		void setDomainIdentifiers(const list<string>& domainIdentifiers);

	private:
		list<string> domainIdentifiers;
	};

} /* namespace Alica */

#endif /* QUANTIFIER_H_ */
