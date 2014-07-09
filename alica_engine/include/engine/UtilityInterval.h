/*
 * UtilityInterval.h
 *
 *  Created on: Jul 9, 2014
 *      Author: Stefan Jakob
 */

#ifndef UTILITYINTERVAL_H_
#define UTILITYINTERVAL_H_

using namespace std;

namespace alica
{

	struct UtilityInterval
	{
	public:
		UtilityInterval(double min, double max);
		virtual ~UtilityInterval();
		double getMax();
		void setMax(double max);
		double getMin();
		void setMin(double min);

	private:
		double min;
		double max;
	};

} /* namespace alica */

#endif /* UTILITYINTERVAL_H_ */
