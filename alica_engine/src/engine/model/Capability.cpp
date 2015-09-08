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

	/**
	 * Computes the similarity between two capability values.
	 * @param roleVal Role value
	 * @param robotVal Robot value
	 * @return The value, ranges between 0 and 1.
	 */
	double Capability::similarityValue(CapValue* roleVal, CapValue* robotVal)
	{
		const int nCount = capValues.size();

		int rlIndex = -1;
		int rbIndex = -1;
		int index = 0;

		// determine the index of both given capability values
		for (auto cap : capValues)
		{
			if (cap == roleVal)
			{
				rlIndex = index;
			}
			if (cap == robotVal)
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

		if (nCount == 1)
		{
			// we found both values and there is only one value, so both must be the same
			return 1;
		}

		// this won't work, in case of only one value (nCount=1), therefore extra handling above
		return (nCount - 1 - abs(rlIndex - rbIndex)) / (nCount - 1);
	}

//====================== Getter and Setter ==============================

	list<CapValue*>& Capability::getCapValues()
	{
		return capValues;
	}
}

