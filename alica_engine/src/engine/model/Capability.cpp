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


		int rlIndex = -1;
		int rbIndex = -1;
		int index = 0;

		for (list<CapValue*>::const_iterator itRoleVal = capValues.begin(); itRoleVal != capValues.end(); itRoleVal++)
		{
			CapValue* cap = *itRoleVal;
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

