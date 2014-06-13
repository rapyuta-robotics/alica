/*
 * Capability.h
 *
 *  Created on: May 28, 2014
 *      Author: snook
 */

#ifndef CAPABILITY_H_
#define CAPABILITY_H_

using namespace std;

#include <list>
#include <iostream>
#include <exception>

#include "AlicaElement.h"

namespace alica
{
	class CapValue;

	class Capability : public AlicaElement
	{
	public:
		Capability();
		virtual ~Capability();

		double similarityValue(CapValue* roleVal, CapValue* robotVal);

		list<CapValue*>& getCapValues() ;


	protected:
		list<CapValue*> capValues;
	};
}
#endif /* CAPABILITY_H_ */
