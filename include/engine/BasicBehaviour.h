/*
 * BasicBehaviour.h
 *
 *  Created on: Jun 4, 2014
 *      Author: stefan
 */

#ifndef BASICBEHAVIOUR_H_
#define BASICBEHAVIOUR_H_

namespace alica
{

	class BasicBehaviour
	{
	public:
		BasicBehaviour();
		virtual ~BasicBehaviour();
	protected:
		int getOwnId();
	};

} /* namespace alica */

#endif /* BASICBEHAVIOUR_H_ */
