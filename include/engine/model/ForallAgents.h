/*
 * ForallAgents.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef FORALLAGENTS_H_
#define FORALLAGENTS_H_

#include "Quantifier.h"

namespace alica
{

	class ForallAgents : public Quantifier
	{
	public:
		ForallAgents(long id = 0);
		virtual ~ForallAgents();
	};

} /* namespace Alica */

#endif /* FORALLAGENTS_H_ */
