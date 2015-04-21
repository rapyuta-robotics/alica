/*
 * Capability.h
 *
 *  Created on: May 28, 2014
 *      Author: Paul Panin
 */

#ifndef CAPABILITY_H_
#define CAPABILITY_H_


#include <list>
#include <iostream>
#include <exception>


#include "AlicaElement.h"

using namespace std;
namespace alica
{
	class CapValue;

	/**
	 * A capability is used to match agents to roles.
	 */
	class Capability : public AlicaElement
	{
	public:
		Capability();
		virtual ~Capability();

		double similarityValue(CapValue* roleVal, CapValue* robotVal);

		list<CapValue*>& getCapValues() ;


	protected:
		/**
		 * List of possible values for this capability
		 */
		list<CapValue*> capValues;
	};
}
#endif /* CAPABILITY_H_ */
