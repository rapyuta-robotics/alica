/*
 * Sum.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef SUM_H_
#define SUM_H_

#include "Term.h"

#include <vector>

namespace AutoDiff
{

	class Sum : public Term
	{
	public:
		Sum(std::vector<Term> terms);

	private:
		std::vector<Term> _terms;
	};

} /* namespace AutoDiff */

#endif /* SUM_H_ */
