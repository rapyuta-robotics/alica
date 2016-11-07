/*
 * CNVec2DAllo.cpp
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#include "container/CNVec2DAllo.h"
#include <sstream>

#include "container/CNVec2DEgo.h"

namespace geometry
{

CNVec2DAllo::CNVec2DAllo(double x, double y)
{
	this->x = x;
	this->y = y;
}

CNVec2DAllo::~CNVec2DAllo() {}

string CNVec2DAllo::toString()
{
    stringstream ss;
    ss << "CNVec2DAllo: x: " << this->x << " y: " << this->y << endl;
    return ss.str();
}

shared_ptr<CNVec2DEgo> CNVec2DAllo::toEgo(CNPositionAllo &me)
{
    return nullptr; // TODO
}

} /* namespace geometry */
