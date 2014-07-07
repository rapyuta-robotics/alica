/*
 * EpByTaskComparer.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef EPBYTASKCOMPARER_H_
#define EPBYTASKCOMPARER_H_

namespace alica
{

	class EntryPoint;

	class EpByTaskComparer
	{
	public:
		EpByTaskComparer();
		virtual ~EpByTaskComparer();
		static bool compareTo(EntryPoint* x, EntryPoint* y);
	};

} /* namespace alica */

#endif /* EPBYTASKCOMPARER_H_ */
