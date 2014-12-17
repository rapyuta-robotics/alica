/*
 * TestConstantValueSummand.h
 *
 *  Created on: Dec 11, 2014
 *      Author: Paul Panin
 */

#ifndef TESTCONSTANTVALUESUMMAND_H_
#define TESTCONSTANTVALUESUMMAND_H_

#include <engine/USummand.h>

namespace alica {

class TestConstantValueSummand: public USummand {
public:
	TestConstantValueSummand(double weight, string name, long id, double val);
	virtual ~TestConstantValueSummand();
	void cacheEvalData();
	UtilityInterval* eval(IAssignment* ass);
	long robotId;

protected:
	double val;
};

} /* namespace alica */

#endif /* TESTCONSTANTVALUESUMMAND_H_ */
