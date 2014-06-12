/*
 * TermBuilder.h
 *
 *  Created on: Jun 6, 2014
 *      Author: psp
 */

#ifndef TERMBUILDER_H_
#define TERMBUILDER_H_

#include "Exp.h"
#include "Product.h"
#include "Sum.h"
#include "Term.h"

#include <vector>

namespace AutoDiff
{

	class TermBuilder
	{
	public:
		static Sum sum(Term v1, Term v2, std::vector<Term> rest = std::vector<Term>());
		static Product product(Term v1, Term v2, std::vector<Term> rest = std::vector<Term>());
		static Exp exp(Term arg);
	};

} /* namespace AutoDiff */

#endif /* TERMBUILDER_H_ */
