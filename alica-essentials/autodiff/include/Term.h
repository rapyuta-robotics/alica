/*
 * Term.h
 *
 *  Created on: Jun 5, 2014
 *      Author: psp
 */

#ifndef TERM_H_
#define TERM_H_

//#include "ITermVisitor.h"

namespace AutoDiff
{
	class Term
	{
	public:
		Term();
		virtual ~Term();

		/**
		 * Accepts a term visitor
		 *
		 * @param visitor The term visitor to accept
		 */
//		virtual void accept(ITermVisitor visitor) = 0;
	};

	Term operator+(const Term& left, const Term& right);
	Term operator*(const Term& left, const Term& right);
//	Term operator/(const Term& numerator, const Term& denominator);
	Term operator-(const Term& left, const Term& right);
} /* namespace AutoDiff */

#endif /* TERM_H_ */
