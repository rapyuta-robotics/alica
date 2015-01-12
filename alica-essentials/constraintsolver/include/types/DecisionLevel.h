/*
 * DecisionLevel.h
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#ifndef DECISIONLEVEL_H_
#define DECISIONLEVEL_H_

namespace alica
{
	namespace reasoner
	{
		namespace cnsat
		{

			class DecisionLevel
			{
			public:
				DecisionLevel(int level);
				virtual ~DecisionLevel();

				int level;
			};

		} /* namespace cnsat */
	} /* namespace reasoner */
} /* namespace alica */

#endif /* DECISIONLEVEL_H_ */
