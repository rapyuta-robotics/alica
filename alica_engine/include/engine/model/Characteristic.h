/*
 * Characteristic.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef CHARACTERISTIC_H_
#define CHARACTERISTIC_H_

using namespace std;

#include <string.h>
#include <iostream>

#include "AlicaElement.h"
#include "Capability.h"
#include "CapValue.h"

namespace alica
{

	class Capability;
	class CapValue;

	/**
	 * A characteristic encapsulates a Capability and aCapValue.
	 */
	class Characteristic : public AlicaElement
	{
	public:
		Characteristic();
		virtual ~Characteristic();

		string toString();

		Capability* getCapability();
		void setCapability(Capability* capability);
		CapValue* getCapValue();
		void setCapValue(CapValue* capValue);
		double getWeight() const;
		void setWeight(double weight);

	protected:
		Capability* capability;
		CapValue* capValue;
		/**
		 * The weight, used for role allocation.
		 */
		double weight = 0;

	};

} /* namespace Alica */

#endif /* CHARACTERISTIC_H_ */
