/*
 * CNVelocity2DEgo.cpp
 *
 *  Created on: 24.12.2016
 *      Author: Philipp Mandler
 */

#include "container/CNVec2D.h"
#include "container/CNPosition.h"
#include "container/CNVelocity2DAllo.h"
#include <sstream>

using namespace std;

namespace geometry
{

CNVelocity2DEgo::~CNVelocityEgo() {}

string CNVelocity2DEgo::toString()
{
    stringstream ss;
    ss << "CNVelocity2DEgo: x: " << this->x << " y: " << this->y << endl;
    return ss.str();
}

shared_ptr<CNVelocity2DAllo> CNVelocity2DAllo::toAllo(CNPositionAllo &me)
{
    return null; // TODO
}

} /* namespace geometry */
