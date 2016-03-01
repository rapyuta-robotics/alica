/*
 * Sum.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "Sum.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "Zero.h"

#include <cmath>

namespace autodiff
{
	/**
	 *
	 */
	Sum::Sum(vector<shared_ptr<Term>> terms) :
			Term()
	{
		this->terms = terms;
	}

	Sum::Sum(shared_ptr<Term> first, shared_ptr<Term> second, vector<shared_ptr<Term>> rest) :
			Term()
	{
		vector<shared_ptr<Term>> terms = {first, second};
		terms.insert(terms.end(), rest.begin(), rest.end());
		this->terms = terms;
	}

	int Sum::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Sum> thisCasted = dynamic_pointer_cast<Sum>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Sum::aggregateConstants()
	{
		shared_ptr<Term> curSummand;
		bool foundConst = false;
		double sum = 0;
		vector<shared_ptr<Term>> nonConstTerms;
		for (int i = 0; i < terms.size(); ++i)
		{
			curSummand = terms[i]->aggregateConstants();
			if (dynamic_pointer_cast<Constant>(curSummand) != 0)
			{
				sum += dynamic_pointer_cast<Constant>(curSummand)->value;
				foundConst = true;
			}
			else
			{
				if (!(dynamic_pointer_cast<Zero>(curSummand) != 0))
				{
					nonConstTerms.push_back(curSummand);
				}
			}
		}
		if (nonConstTerms.size() == 0)
		{
			return TermBuilder::constant(sum);
		}
		else if (!foundConst && nonConstTerms.size() == 1)
		{
			return nonConstTerms[0];
		}
		if (foundConst)
		{
			nonConstTerms.push_back(TermBuilder::constant(sum));
		}
		terms.clear();
		for (auto term : nonConstTerms)
		{
			terms.push_back(term);
		}
		return shared_from_this();
	}

	shared_ptr<Term> Sum::derivative(shared_ptr<Variable> v)
	{
		vector<shared_ptr<Term>> t;
		for (int i = 0; i < terms.size(); ++i)
		{
			t.push_back(terms[i]->derivative(v));
		}
		return make_shared<Sum>(t);
	}

	string Sum::toString()
	{
		string str;
		str.append("( ");
		str.append(terms[0]->toString());
		for (int i = 1; i < terms.size(); ++i)
		{
			str.append(" + ");
			str.append(terms[i]->toString());
		}
		str.append(" )");
		return str;
	}
} /* namespace autodiff */
