/*
 * DecisionLevel.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#include "types/DecisionLevel.h"

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{

			DecisionLevel::DecisionLevel(int level)
			{
				this->level = level;
			}

			DecisionLevel::~DecisionLevel()
			{
				// TODO Auto-generated destructor stub
			}

		} /* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */
