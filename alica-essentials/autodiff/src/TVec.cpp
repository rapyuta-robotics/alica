/*
 * TVec.cpp
 *
 *  Created on: Aug 12, 2014
 *      Author: psp
 */

#include "TVec.h"
#include <cstdarg>
#include "Term.h"
#include "TermBuilder.h"

namespace autodiff
{
	TVec::TVec(vector<shared_ptr<Term>> terms)
	{
		this->terms = terms;
	}

//	TVec::TVec(shared_ptr<Term> terms, ...)
//	{
//		this->terms = vector<shared_ptr<Term>>();
//		va_list args;
//		va_start(args, terms);
//
//		shared_ptr<Term> currentTerm = terms;
//
//		do {
//			this->terms.push_back(currentTerm);
//		} while((currentTerm = va_arg(terms, shared_ptr<Term>)) != NULL);
//
//		va_end(args);
//	}



	TVec::TVec(initializer_list<double> values)
	{
		terms = vector<shared_ptr<Term>>(values.size());
		std::initializer_list<double>::iterator it;
		int i = 0;
		for (it = values.begin(); it != values.end(); ++it, ++i)
		{
			terms[i] = TermBuilder::constant(*it);
		}
	}

	TVec::TVec(shared_ptr<TVec> first, vector<shared_ptr<Term>> rest)
	{
		vector<shared_ptr<Term>> terms = first->terms;
		terms.insert(terms.end(), rest.begin(), rest.end());
		this->terms = terms;
	}

	TVec::TVec(vector<shared_ptr<Term>> left, vector<shared_ptr<Term>> right,
				function<shared_ptr<Term>(shared_ptr<Term>, shared_ptr<Term>)> elemOp)
	{
		terms = vector<shared_ptr<Term>>(left.size());
		for (int i = 0; i < terms.size(); ++i)
		{
			terms[i] = elemOp(left[i], right[i]);
		}
	}

	TVec::TVec(vector<shared_ptr<Term>> input, function<shared_ptr<Term>(shared_ptr<Term>)> elemOp)
	{
		terms = vector<shared_ptr<Term>>(input.size());
		for (int i = 0; i < input.size(); ++i)
		{
			terms[i] = elemOp(input[i]);
		}
	}

	shared_ptr<Term> TVec::normSquared()
	{
		vector<shared_ptr<Term>> powers;
		for (int i = 0; i < terms.size(); ++i)
		{
			powers.push_back(TermBuilder::power(terms[i], 2));
		}
		return TermBuilder::sum(powers);
	}

	shared_ptr<TVec> TVec::normalize()
	{
		shared_ptr<Term> a = normSquared();
		a = TermBuilder::power(a, 0.5);
		vector<shared_ptr<Term>> b;
		for (int i = 0; i < terms.size(); ++i)
		{
			b.push_back(terms[i] / a);
		}
		return make_shared<TVec>(b);
	}

	int TVec::dimension()
	{
		return terms.size();
	}

	shared_ptr<Term> TVec::getX()
	{
// TODO:		return shared_from_this()[0];
		return terms[0];
	}

	shared_ptr<Term> TVec::getY()
	{
// TODO:		return shared_from_this()[1];
		return terms[1];
	}

	shared_ptr<Term> TVec::getZ()
	{
// TODO:		return shared_from_this()[2];
		return terms[2];
	}

	shared_ptr<Term> TVec::innerProduct(shared_ptr<TVec> left, shared_ptr<TVec> right)
	{
		vector<shared_ptr<Term>> products;
		for (int i = 0; i < left->dimension(); ++i)
		{
			products.push_back(left->terms[i] * right->terms[i]);
		}
		return TermBuilder::sum(products);
	}

	shared_ptr<TVec> TVec::crossProduct(shared_ptr<TVec> left, shared_ptr<TVec> right)
	{
		return make_shared<TVec>(
				initializer_list<shared_ptr<Term>> {left->getY() * right->getZ() - left->getZ() * right->getY(),
													left->getZ() * right->getX() - left->getX() * right->getZ(),
													left->getX() * right->getY() - left->getY() * right->getX()});
	}

	shared_ptr<Term> TVec::operator[](int index)
	{
		return terms[index];
	}

	shared_ptr<TVec> operator+(const shared_ptr<TVec>& left, const shared_ptr<TVec>& right)
	{
		return make_shared<TVec>(left->terms, right->terms, [] (shared_ptr<Term> left, shared_ptr<Term> right)
		{
			return left + right;
		});
	}

	shared_ptr<TVec> operator-(const shared_ptr<TVec>& left, const shared_ptr<TVec>& right)
	{
		return make_shared<TVec>(left->terms, right->terms, [] (shared_ptr<Term> left, shared_ptr<Term> right)
		{
			return left - right;
		});
	}

	shared_ptr<TVec> operator-(const shared_ptr<TVec>& vector)
	{
		return vector * -1;
	}

	shared_ptr<TVec> operator*(const shared_ptr<TVec>& vector, const shared_ptr<Term>& scalar)
	{
		return make_shared<TVec>(vector->terms, [&scalar] (shared_ptr<Term> x)
		{
			return scalar * x;
		});
	}

	shared_ptr<TVec> operator*(const shared_ptr<TVec>& vector, const double scalar)
	{
		return vector * TermBuilder::constant(scalar);
	}

	shared_ptr<TVec> operator*(const shared_ptr<Term>& scalar, const shared_ptr<TVec>& vector)
	{
		return make_shared<TVec>(vector->terms, [&scalar] (shared_ptr<Term> x)
		{
			return scalar * x;
		});
	}

	shared_ptr<TVec> operator*(const double scalar, const shared_ptr<TVec>& vector)
	{
		return TermBuilder::constant(scalar) * vector;
	}

	shared_ptr<Term> operator*(const shared_ptr<TVec>& left, const shared_ptr<TVec>& right)
	{
		return TVec::innerProduct(left, right);
	}

} /* namespace autodiff */
