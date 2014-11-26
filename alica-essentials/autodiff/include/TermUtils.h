/*
 * TermUtils.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef TERMUTILS_H_
#define TERMUTILS_H_

#include "ICompiledTerm.h"
#include "Variable.h"

#include <vector>

using namespace std;

namespace autodiff
{

	class TermUtils
	{
	public:
		static shared_ptr<ICompiledTerm> compile(shared_ptr<Term> term, shared_ptr<vector<shared_ptr<Variable>>> variables);
		static double evaluate(shared_ptr<Term> term, shared_ptr<vector<shared_ptr<Variable>>> variables, shared_ptr<vector<double>> point);
		static shared_ptr<vector<double>> differentiate(shared_ptr<Term> term, shared_ptr<vector<shared_ptr<Variable>>> variables, shared_ptr<vector<double>> point);
	};

} /* namespace autodiff */

#endif /* TERMUTILS_H_ */
