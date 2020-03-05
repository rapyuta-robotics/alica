/*
 * CNVec2DAllo.cpp
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#include <cnc_geometry/CNVecAllo.h>
#include <cnc_geometry/CNVecEgo.h>
#include <sstream>

#include "cnc_geometry/CNPositionAllo.h"

using std::shared_ptr;
using std::string;

namespace geometry {

CNVecAllo::CNVecAllo(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

CNVecAllo::CNVecAllo(const CNVecAllo& obj) {
    this->x = obj.x;
    this->y = obj.y;
    this->z = obj.z;
}

CNVecAllo::~CNVecAllo() {}

string CNVecAllo::toString() const {
    std::stringstream ss;
    ss << "CNVecAllo: x: " << x << " y: " << y << " z: " << z << std::endl;
    return ss.str();
}

CNVecEgo CNVecAllo::toEgo(CNPositionAllo& me) const {
    auto ego = CNVecEgo();

    // rotate rel point around origin -> rel point with ego orientation
    double s = sin(-me.theta);
    double c = cos(-me.theta);

    ego.x = c * this->x - s * this->y;
    ego.y = s * this->x + c * this->y;
    ego.z = this->z;

    return ego;
}

} /* namespace geometry */
