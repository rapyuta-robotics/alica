/*
 * TermBuilder.h
 *
 *  Created on: Jun 6, 2014
 *      Author: psp
 */

#ifndef TERMBUILDER_H_
#define TERMBUILDER_H_

#include <vector>
#include <memory>

using namespace std;

namespace AutoDiff
{
	class Constant;
	class ConstPower;
	class Exp;
	class Product;
	class Sum;
	class Term;

	class TermBuilder
	{
	public:
		static shared_ptr<Term> constant(double value);
		static shared_ptr<Sum> sum(shared_ptr<Term> v1, shared_ptr<Term> v2, std::vector<shared_ptr<Term>> rest = std::vector<shared_ptr<Term>>());
		static shared_ptr<Product> product(shared_ptr<Term> v1, shared_ptr<Term> v2, std::vector<shared_ptr<Term>> rest = std::vector<shared_ptr<Term>>());
		static shared_ptr<ConstPower> power(shared_ptr<Term> t, double power);
		static shared_ptr<Exp> exp(shared_ptr<Term> arg);
	};

} /* namespace AutoDiff */

#endif /* TERMBUILDER_H_ */
