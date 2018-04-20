/*
 * ICompiledTerm.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef ICOMPILEDTERM_H_
#define ICOMPILEDTERM_H_

#include <vector>
#include <utility>
#include <memory>

using namespace std;

namespace autodiff {

class ICompiledTerm {
public:
    virtual ~ICompiledTerm() {}

    /**
     * Evaluates the compiled term at the given point.
     * <remarks>The number at <c>arg[i]</c> is the value assigned to the variable <c>Variables[i]</c>.</remarks>
     *
     * @param arg The point at which to evaluate.
     *
     * @return The value of the function represented by the term at the given point.
     */
    virtual double evaluate(shared_ptr<vector<double>> arg) = 0;

    /**
     * Computes the gradient of the compiled term at the given point.
     * <remarks>The number at <c>arg[i]</c> is the value assigned to the variable <c>Variables[i]</c>.</remarks>
     *
     * @params arg The point at which to differentiate.
     *
     * @return A tuple, where the first item is the gradient at arg and the second item is the value at arg1. That is,
     * the second value is the same as running evaluate on arg.
     */
    virtual pair<shared_ptr<vector<double>>, double> differentiate(shared_ptr<vector<double>> arg) = 0;

    /**
     * The collection of variables contained in this compiled term.
     * <remarks>
     * The order of variables in this collection specifies the meaning of each argument in <see cref="Differentiate"/>
     * or <see cref="Evaluate"/>. That is, the variable at <c>Variables[i]</c> corresponds to the i-th parameter of <see
     * cref="Differentiate"/> and <see cref="Evaluate"/>.
     * </remarks>
     */
    // TODO:		ReadOnlyCollection<Variable> Variables { get; }
};

} /* namespace autodiff */

#endif /* ICOMPILEDTERM_H_ */
