/*
 * Capability.cpp
 *
 *  Created on: May 28, 2014
 *      Author: Paul Panin
 */

#include "engine/model/Capability.h"
#include "engine/model/CapValue.h"

namespace alica
{
	Capability::Capability()
	{
	}

	Capability::~Capability()
	{
	}

	double Capability::similarityValue(CapValue* roleVal, CapValue* robotVal)
	{
		const int nCount = capValues.size();

		for(CapValue* c : capValues)
		{
			cout << "NAME " << c << endl;
		}

		int rlIndex = -1;
		int rbIndex = -1;
		int index = 0;
		cout << "ROLEVAL " << roleVal->getName() << " RobotVal " << robotVal->getName() << endl;

		for (list<CapValue*>::const_iterator itRoleVal = capValues.begin(); itRoleVal != capValues.end(); itRoleVal++)
		{
			cout << "ITER " << *itRoleVal << roleVal << robotVal << endl;
			CapValue* cap = *itRoleVal;
			cout << cap->getId() << " " << roleVal->getId() << " " <<robotVal->getId() << endl;
			if (*itRoleVal == roleVal)
			{
				rlIndex = index;
			}
			if (*itRoleVal == robotVal)
			{
				rbIndex = index;
			}
			++index;
		}

		if (rlIndex == -1)
		{
			cout << "Capability::similarityValue: Role not found!" << endl;
			throw exception();
		}
		if (rbIndex == -1)
		{
			cout << "Capability::similarityValue: Robot not found!" << endl;
			throw exception();
		}
		return (nCount - 1 - abs(rlIndex - rbIndex)) / (nCount - 1);
	}

//====================== Getter and Setter ==============================

	list<CapValue*>& Capability::getCapValues()
	{
		return capValues;
	}
}

