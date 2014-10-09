/*
 * IConstraintCreator.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_ENGINE_INCLUDE_ENGINE_ICONSTRAINTCREATOR_H_
#define ALICA_ALICA_ENGINE_INCLUDE_ENGINE_ICONSTRAINTCREATOR_H_

#include <memory>

using namespace std;

namespace alica
{
	class BasicConstraint;

	class IConstraintCreator
	{
	public:
		virtual ~IConstraintCreator(){}
		virtual shared_ptr<BasicConstraint> createConstraint(long constraintConfId) = 0;
	};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_INCLUDE_ENGINE_ICONSTRAINTCREATOR_H_ */
