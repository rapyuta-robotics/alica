/*
 * Exp.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef EXP_H_
#define EXP_H_

#include "Term.h"

namespace AutoDiff
{

	class Exp : public Term
	{
	public:
		Exp(Term arg);

	private:
		Term _arg;
	};

} /* namespace AutoDiff */

#endif /* EXP_H_ */
