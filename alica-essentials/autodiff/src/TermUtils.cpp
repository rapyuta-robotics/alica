/*
 * TermUtils.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "TermUtils.h"

#include "CompiledDifferentiator.h"

namespace autodiff
{
	/**
	 * Creates a compiled representation of a given term that allows efficient evaluation of the value/gradient.
	 * <remarks>
	 * The order of the variables in <paramref name="variables"/> is important. Each call to <c>ICompiledTerm.Evaluate</c> or
	 * <c>ICompiledTerm.Differentiate</c> receives an array of numbers representing the point of evaluation. The i'th number in this array corresponds
	 * to the i'th variable in <c>variables</c>.
	 * </remarks>
	 *
	 * @param term The term to compile.
	 * @param variables The variables contained in the term.
	 *
	 * @return A compiled representation of term that assigns values to variables in the same order as in variables
	 */
	shared_ptr<ICompiledTerm> TermUtils::compile(shared_ptr<Term> term, shared_ptr<vector<shared_ptr<Variable>>> variables) {
		return make_shared<CompiledDifferentiator>(term, variables);
	}

	/**
	 * Evaluates the function represented by a given term at a given point.
	 * <remarks>The i'th value in <c>point</c> corresponds to the i'th variable in <c>variables</c>.</remarks>
	 *
	 * @param term The term representing the function to evaluate.
	 * @param variables The variables used in term.
	 * @param point The values assigned to the variables in variables
	 *
	 * @return The value of the function represented by term at the point represented by variables and point.
	 */
	double TermUtils::evaluate(shared_ptr<Term> term, shared_ptr<vector<shared_ptr<Variable>>> variables, shared_ptr<vector<double>> point)
	{
		return compile(term, variables)->evaluate(point);
	}

	/**
	 * Computes the gradient of the function represented by a given term at a given point.
	 * <remarks>The i'th value in <c>point</c> corresponds to the i'th variable in <c>variables</c>. In addition, the i'th value
	 * in the resulting array is the partial derivative with respect to the i'th variable in <c>variables</c>.</remarks>
	 *
	 * @param term The term representing the function to differentiate.
	 * @param variables The variables used in term.
	 * @param point The values assigned to the variables in variables
	 *
	 * @return The gradient of the function represented by term at the point represented by variables and point.
	 */
	shared_ptr<vector<double>> TermUtils::differentiate(shared_ptr<Term> term, shared_ptr<vector<shared_ptr<Variable>>> variables, shared_ptr<vector<double>> point) {
		shared_ptr<vector<double>> result = compile(term, variables)->differentiate(point).first;
		return result;
	}
} /* namespace autodiff */
