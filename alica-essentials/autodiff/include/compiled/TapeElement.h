/*
 * TapeElement.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef TAPEELEMENT_H_
#define TAPEELEMENT_H_

#include <vector>
#include <memory>

using namespace std;

namespace AutoDiff
{
	namespace Compiled
	{
		struct InputEdge
		{
			int index;
			double weight;
		};

		class ITapeVisitor;

		class TapeElement : public enable_shared_from_this<TapeElement>
		{
		public:
			virtual ~TapeElement()
			{
			}

			double value;
			double adjoint;
			vector<InputEdge> inputs;

			virtual void accept(shared_ptr<ITapeVisitor> visitor) = 0;
		};
	} // namespace Compiled
} /* namespace AutoDiff */

#endif /* TAPEELEMENT_H_ */
