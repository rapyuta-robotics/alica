/*
 * TVec.h
 *
 *  Created on: Aug 12, 2014
 *      Author: psp
 */

#ifndef TVEC_H_
#define TVEC_H_

#include <memory>
#include <vector>

using namespace std;

namespace AutoDiff
{
	class Term;

	class TVec : public enable_shared_from_this<TVec>
	{
	public:
		TVec(vector<shared_ptr<Term>> terms);
		TVec(shared_ptr<TVec> first, vector<shared_ptr<Term>> rest);
		TVec(vector<shared_ptr<Term>> left, vector<shared_ptr<Term>> right,
				function<shared_ptr<Term>(shared_ptr<Term>, shared_ptr<Term>)> elemOp);
		TVec(vector<shared_ptr<Term>> input, function<shared_ptr<Term>(shared_ptr<Term>)> elemOp);

		shared_ptr<Term> normSquared();
		shared_ptr<TVec> normalize();
		int dimension();
		shared_ptr<Term> getX();
		shared_ptr<Term> getY();
		shared_ptr<Term> getZ();
		vector<shared_ptr<Term>> getTerms();
		static shared_ptr<Term> innerProduct(shared_ptr<TVec> left, shared_ptr<TVec> right);
		static shared_ptr<TVec> crossProduct(shared_ptr<TVec> left, shared_ptr<TVec> right);

		shared_ptr<Term> operator[](int index);
	private:
		vector<shared_ptr<Term>> _terms;
	};

	shared_ptr<TVec> operator+(const shared_ptr<TVec>& left, const shared_ptr<TVec>& right);
	shared_ptr<TVec> operator-(const shared_ptr<TVec>& left, const shared_ptr<TVec>& right);
	shared_ptr<TVec> operator-(const shared_ptr<TVec>& vector);
	shared_ptr<TVec> operator*(const shared_ptr<TVec>& vector, const shared_ptr<Term>& scalar);
	shared_ptr<TVec> operator*(const shared_ptr<TVec>& vector, const double scalar);
	shared_ptr<TVec> operator*(const shared_ptr<Term>& scalar, const shared_ptr<TVec>& vector);
	shared_ptr<TVec> operator*(const double scalar, const shared_ptr<TVec>& vector);
	shared_ptr<Term> operator*(const shared_ptr<TVec>& left, const shared_ptr<TVec>& right);

} /* namespace AutoDiff */

#endif /* TVEC_H_ */
