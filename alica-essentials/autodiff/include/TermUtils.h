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

namespace AutoDiff
{

	class TermUtils
	{
	public:
		static shared_ptr<ICompiledTerm> compile(shared_ptr<Term> term, vector<shared_ptr<Variable>> variables);
		static double evaluate(shared_ptr<Term> term, vector<shared_ptr<Variable>> variables, vector<double> point);
		static vector<double> differentiate(shared_ptr<Term> term, vector<shared_ptr<Variable>> variables, vector<double> point);
	};

} /* namespace AutoDiff */

#endif /* TERMUTILS_H_ */
