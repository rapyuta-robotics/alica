/*
 * TVec.cpp
 *
 *  Created on: Aug 12, 2014
 *      Author: psp
 */

#include "TVec.h"

#include "Term.h"
#include "TermBuilder.h"

namespace AutoDiff
{
	TVec::TVec(vector<shared_ptr<Term>> terms)
	{
		_terms = terms;
	}

	TVec::TVec(shared_ptr<TVec> first, vector<shared_ptr<Term>> rest) //:
//			TVec(vector<shared_ptr<Term>>(first->getTerms(), rest))
	{
		vector<shared_ptr<Term>> terms = first->getTerms();
		terms.insert(terms.end(), rest.begin(), rest.end());
		_terms = terms;
		// public TVec(TVec first, params Term[] rest)
		// : this(first.terms.Concat(rest ?? System.Linq.Enumerable.Empty<Term>()))
	}

	TVec::TVec(vector<shared_ptr<Term>> left, vector<shared_ptr<Term>> right,
				function<shared_ptr<Term>(shared_ptr<Term>, shared_ptr<Term>)> elemOp)
	{
		_terms = vector<shared_ptr<Term>>(left.size());
		for (int i = 0; i < _terms.size(); ++i)
		{
			_terms[i] = elemOp(left[i], right[i]);
		}
	}

	TVec::TVec(vector<shared_ptr<Term>> input, function<shared_ptr<Term>(shared_ptr<Term>)> elemOp)
	{
		_terms = vector<shared_ptr<Term>>(input.size());
		for (int i = 0; i < input.size(); ++i)
		{
			_terms[i] = elemOp(input[i]);
		}
	}

	TVec::TVec(shared_ptr<Term> x)
	{
		_terms = vector<shared_ptr<Term>>();
		_terms.push_back(x);
	}

	TVec::TVec(shared_ptr<Term> x, shared_ptr<Term> y) :
			TVec(x)
	{
		_terms.push_back(y);
	}

	TVec::TVec(shared_ptr<Term> x, shared_ptr<Term> y, shared_ptr<Term> z) :
			TVec(x, y)
	{
		_terms.push_back(z);
	}

	TVec::TVec(double x) :
			TVec(TermBuilder::constant(x))
	{

	}

	TVec::TVec(double x, double y) :
			TVec(TermBuilder::constant(x), TermBuilder::constant(y))
	{

	}
	TVec::TVec(double x, double y, double z) :
			TVec(TermBuilder::constant(x), TermBuilder::constant(y), TermBuilder::constant(z))
	{

	}

	shared_ptr<Term> TVec::normSquared()
	{
		vector<shared_ptr<Term>> powers;
		for (int i = 0; i < _terms.size(); ++i)
		{
			powers.push_back(TermBuilder::power(_terms[i], 2));
		}
		return TermBuilder::sum(powers);
	}

	shared_ptr<TVec> TVec::normalize()
	{
		shared_ptr<Term> a = normSquared();
		a = TermBuilder::power(a, 0.5);
		vector<shared_ptr<Term>> b;
		for (int i = 0; i < _terms.size(); ++i)
		{
			b.push_back(_terms[i] / a);
		}
		return make_shared<TVec>(b);
	}

	int TVec::dimension()
	{
		return _terms.size();
	}

	shared_ptr<Term> TVec::getX()
	{
// TODO:		return shared_from_this()[0];
		return _terms[0];
	}

	shared_ptr<Term> TVec::getY()
	{
// TODO:		return shared_from_this()[1];
		return _terms[1];
	}

	shared_ptr<Term> TVec::getZ()
	{
// TODO:		return shared_from_this()[2];
		return _terms[2];
	}

	vector<shared_ptr<Term>> TVec::getTerms()
	{
		return _terms;
	}

	shared_ptr<Term> TVec::innerProduct(shared_ptr<TVec> left, shared_ptr<TVec> right)
	{
		vector<shared_ptr<Term>> products;
		for (int i = 0; i < left->dimension(); ++i)
		{
			products.push_back(left->getTerms()[i] * right->getTerms()[i]);
		}
		return TermBuilder::sum(products);
	}

	shared_ptr<TVec> TVec::crossProduct(shared_ptr<TVec> left, shared_ptr<TVec> right)
	{
		vector<shared_ptr<Term>> terms;
		terms.push_back(left->getY() * right->getZ() - left->getZ() * right->getY());
		terms.push_back(left->getZ() * right->getX() - left->getX() * right->getZ());
		terms.push_back(left->getX() * right->getY() - left->getY() * right->getX());
		return make_shared<TVec>(terms);
//		return make_shared<TVec>(left->getY() * right->getZ() - left->getZ() * right->getY(),
//									left->getZ() * right->getX() - left->getX() * right->getZ(),
//									left->getX() * right->getY() - left->getY() * right->getX());
	}

	shared_ptr<Term> TVec::operator[](int index)
	{
		return _terms[index];
	}

	shared_ptr<TVec> operator+(const shared_ptr<TVec>& left, const shared_ptr<TVec>& right)
	{
		return make_shared<TVec>(left->getTerms(), right->getTerms(), [] (shared_ptr<Term> left, shared_ptr<Term> right)
		{
			return left + right;
		});
	}

	shared_ptr<TVec> operator-(const shared_ptr<TVec>& left, const shared_ptr<TVec>& right)
	{
		return make_shared<TVec>(left->getTerms(), right->getTerms(), [] (shared_ptr<Term> left, shared_ptr<Term> right)
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
		return make_shared<TVec>(vector->getTerms(), [&scalar] (shared_ptr<Term> x)
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
		return make_shared<TVec>(vector->getTerms(), [&scalar] (shared_ptr<Term> x)
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

} /* namespace AutoDiff */
