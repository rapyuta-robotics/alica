/*
 * IParametricCompiledTerm.h
 *
 *  Created on: Aug 12, 2014
 *      Author: psp
 */

#ifndef IPARAMETRICCOMPILEDTERM_H_
#define IPARAMETRICCOMPILEDTERM_H_

#include <vector>
#include <utility>

using namespace std;

namespace autodiff
{

	class IParametricCompiledTerm
	{
	public:
		virtual ~IParametricCompiledTerm()
		{
		}

		/**
		 * Evaluates the compiled term at the given point.
		 * <remarks>The number at <c>arg[i]</c> is the value assigned to the variable <c>Variables[i]</c>.</remarks>
		 *
		 * @param arg The point at which to evaluate..
		 * @param parameters The parameter values
		 *
		 * @return The value of the function represented by the term at the given point.
		 */
		virtual double evaluate(vector<double> arg, vector<double> parameters) = 0;

		/**
		 * Computes the gradient of the compiled term at the given point.
		 * <remarks>The number at <c>arg[i]</c> is the value assigned to the variable <c>Variables[i]</c>.</remarks>
		 *
		 * @param arg The point at which to differentiate.
		 * @param parameters The parameter values
		 *
		 * @return A tuple, where the first item is the gradient at arg and the second item is the value at arg. That is, the second value is the same as running evaluate on arg and parameters.
		 */
		virtual pair<vector<double>, double> differentiate(vector<double> arg, vector<double> parameters) = 0;

		/**
		 * The collection of variables contained in this compiled term.
		 * <remarks>
		 * The order of variables in this collection specifies the meaning of each argument in <see cref="Differentiate"/> or
		 * <see cref="Evaluate"/>. That is, the variable at <c>Variables[i]</c> corresponds to the i-th element in the <c>arg</c> parameter of <see cref="Differentiate"/>
		 * and <see cref="Evaluate"/>.
		 * </remarks>
		 */
// TODO:		ReadOnlyCollection<Variable> Variables { get; }
		/**
		 * The collection of parameter variables contained in this compiled term.
		 * <remarks>
		 * The order of variables in this collection specifies the meaning of each argument in <see cref="Differentiate"/> or
		 * <see cref="Evaluate"/>. That is, the variable at <c>Variables[i]</c> corresponds to the i-th element in the <c>parameters</c> parameter of <see cref="Differentiate"/>
		 * and <see cref="Evaluate"/>.
		 * </remarks>
		 */
// TODO:		ReadOnlyCollection<Variable> Parameters { get; }

		// TODO:
//		class ParametricCompiledTermContract : public IParametricCompiledTerm
//		{
//		public:
//			double evaluate(vector<double> arg, vector<double> parameters)
//			{
//
//			}
//
//			pair<vector<double>, double> differentiate(vector<double> arg, vector<double> parameters)
//			{
//
//			}
//		};
	};

} /* namespace autodiff */

#endif /* IPARAMETRICCOMPILEDTERM_H_ */
