/*
 * Term.h
 *
 *  Created on: Jun 5, 2014
 *      Author: psp
 */

#ifndef TERM_H_
#define TERM_H_

#include "ITermVisitor.h"

#include <memory>
#include <vector>

using namespace std;

namespace AutoDiff
{
	enum AndType {
		MIN,
		AND
	};
	enum OrType {
		MAX,
		OR
	};

	class Term : public enable_shared_from_this<Term>
	{
	public:
		Term();
		virtual ~Term();

		/**
		 * Accepts a term visitor
		 *
		 * @param visitor The term visitor to accept
		 */
		virtual int accept(shared_ptr<ITermVisitor> visitor) = 0;

		int getIndex() const;

		// Additions by Carpe Noctem:
		double _min;
		double _max;

		shared_ptr<Term> _prev;
		shared_ptr<Term> _next;

		vector<shared_ptr<Term>> parents;

		// Extension to Term for fuzzy constraints:
		static const shared_ptr<Term> TRUE;
		static const shared_ptr<Term> FALSE;
		static const double EPSILON;

		virtual shared_ptr<Term> aggregateConstants() = 0;
		virtual shared_ptr<Term> derivative(shared_ptr<Variable> v) = 0;
		virtual shared_ptr<Term> negate();

		static AndType getAnd();
		static void setAnd(AndType a);
		static OrType getOr();
		static void setOr(OrType o);
		static double getConstraintSteepness();
		static void setConstraintSteepness(double constraintSteepness);

//		bool operator==(const shared_ptr<Term>& term);
		friend bool operator==(const shared_ptr<Term>& left, const shared_ptr<Term>& right);
//		friend bool operator!=(const shared_ptr<Term>& left, const shared_ptr<Term>& right);
	private:
		const int m_id;
		static int m_nextId;

		static OrType _orop;
		static AndType _andop;

		static double _constraintSteepness;
	};

	shared_ptr<Term> operator+(const shared_ptr<Term>& left, const shared_ptr<Term>& right);
	shared_ptr<Term> operator*(const shared_ptr<Term>& left, const shared_ptr<Term>& right);
	shared_ptr<Term> operator/(const shared_ptr<Term>& numerator, const shared_ptr<Term>& denominator);
	shared_ptr<Term> operator-(const shared_ptr<Term>& left, const shared_ptr<Term>& right);

	shared_ptr<Term> operator+(const double left, const shared_ptr<Term>& right);
	shared_ptr<Term> operator*(const double left, const shared_ptr<Term>& right);
	shared_ptr<Term> operator/(const double numerator, const shared_ptr<Term>& denominator);
	shared_ptr<Term> operator-(const double left, const shared_ptr<Term>& right);

	shared_ptr<Term> operator+(const shared_ptr<Term>& left, const double right);
	shared_ptr<Term> operator*(const shared_ptr<Term>& left, const double right);
	shared_ptr<Term> operator/(const shared_ptr<Term>& numerator, const double denominator);
	shared_ptr<Term> operator-(const shared_ptr<Term>& left, const double right);

	shared_ptr<Term> operator-(const shared_ptr<Term>& term);

	shared_ptr<Term> operator!(const shared_ptr<Term>& term);
	shared_ptr<Term> operator&(const shared_ptr<Term>& left, const shared_ptr<Term>& right);
	shared_ptr<Term> operator|(const shared_ptr<Term>& left, const shared_ptr<Term>& right);
	shared_ptr<Term> operator%(const shared_ptr<Term>& left, const shared_ptr<Term>& right);
	shared_ptr<Term> operator^(const shared_ptr<Term>& left, const shared_ptr<Term>& right);
} /* namespace AutoDiff */

#endif /* TERM_H_ */
