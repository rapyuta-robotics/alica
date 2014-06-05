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

	class Characteristic : public AlicaElement
	{
	public:
		Characteristic();
		virtual ~Characteristic();

		string toString();

		const Capability& getCapability() const;
		void setCapability(const Capability& capability);
		const CapValue& getCapValue() const;
		void setCapValue(const CapValue& capValue);
		double getWeight() const;
		void setWeight(double weight);

	protected:
		Capability capability;
		CapValue capValue;
		double weight = 0;

	};

} /* namespace Alica */

#endif /* CHARACTERISTIC_H_ */
