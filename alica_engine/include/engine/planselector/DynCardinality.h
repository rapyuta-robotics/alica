/*
 * DynCardinality.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef DYNCARDINALITY_H_
#define DYNCARDINALITY_H_

namespace alica
{

	class DynCardinality
	{
	public:
		DynCardinality(int min, int max);
		virtual ~DynCardinality();
		int getMax();
		void setMax(int max);
		int getMin();
		void setMin(int min);

	protected:
		int min;
		int max;
	};

} /* namespace alica */

#endif /* DYNCARDINALITY_H_ */
