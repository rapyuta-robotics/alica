/*
 * CNVelocity2DAllo.cpp
 *
 *  Created on: 24.12.2016
 *      Author: Philipp Mandler
 */

#include "container/CNVec2D.h"
#include "container/CNPosition.h"
#include "container/CNVelocity2DEgo.h"
#include <sstream>

using namespace std;

namespace geometry
{

CNVelocity2DAllo::~CNVelocityAllo() {}

string CNVelocity2DAllo::toString()
{
    stringstream ss;
    ss << "CNVelocity2DAllo: x: " << this->x << " y: " << this->y << endl;
    return ss.str();
}

shared_ptr<CNVelocity2DEgo> CNVelocity2DAllo::toEgo(CNPositionAllo &me)
{
    return null; // TODO
}

} /* namespace geometry */
