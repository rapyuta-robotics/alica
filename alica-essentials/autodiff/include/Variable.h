/*
 * Variable.h
 *
 *  Created on: Jun 5, 2014
 *      Author: psp
 */

#ifndef AUTODIFFVARIABLE_H_
#define AUTODIFFVARIABLE_H_

#include "Term.h"

namespace AutoDiff
{
	class Variable : public Term
	{
	public:
		Variable();
	};
} /* namespace AutoDiff */

#endif /* VARIABLE_H_ */
