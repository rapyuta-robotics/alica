/*
 * CNVec2DAllo.h
 *
 *  Created on: 07.11.2016
 *      Author: Philipp Mandler
 */

#pragma once

#include "CNVec2DTemplate.h"

namespace geometry
{

class CNVec2DEgo;
class CNPositionAllo;
class CNVec2DEgo;

class CNVec2DAllo : public CNVec2DTemplate<CNVec2DAllo>
{
  public:
	CNVec2DAllo() : CNVec2DAllo(0, 0) {};
	CNVec2DAllo(double x, double y);
    virtual ~CNVec2DAllo();

    std::string toString();

    std::shared_ptr<CNVec2DEgo> toEgo(CNPositionAllo &origin);
};

} /* namespace geometry */
