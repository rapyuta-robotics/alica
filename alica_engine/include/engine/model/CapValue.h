/*
 * CapValue.h
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef CAPVALUE_H_
#define CAPVALUE_H_

#include "AlicaElement.h"

namespace alica {

/**
 * A value for a Capability.
 */
class CapValue : public AlicaElement {
public:
    CapValue();
    virtual ~CapValue();
};

} /* namespace alica */

#endif /* CAPVALUE_H_ */
