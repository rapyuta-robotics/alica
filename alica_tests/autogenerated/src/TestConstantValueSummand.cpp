/*
 * TestConstantValueSummand.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: Paul Panin
 */

#include "TestConstantValueSummand.h"

namespace alica
{

TestConstantValueSummand::TestConstantValueSummand(double weight, std::string name, long id, double val)
    : robotId(nullptr)
{
    this->weight = weight;
    this->name = name;
    this->id = id;
    this->val = val;
}

TestConstantValueSummand::~TestConstantValueSummand() {}
void TestConstantValueSummand::cacheEvalData() {}
UtilityInterval TestConstantValueSummand::eval(IAssignment* ass)
{
    ui.setMin(val);
    ui.setMax(val);

    return ui;
}
} /* namespace alica */
