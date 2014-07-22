/*
 * ITapevisitor.h
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#ifndef ITAPEVISITOR_H_
#define ITAPEVISITOR_H_

#include <memory>

using namespace std;

namespace AutoDiff
{
	namespace Compiled
	{
		class CompiledConstant;
		class CompiledExp;
		class CompiledProduct;
		class CompiledSum;
		class CompiledVariable;

		class ITapeVisitor
		{
		public:
			virtual ~ITapeVisitor()
			{
			}

			virtual void visit(shared_ptr<CompiledConstant> elem) = 0;
			virtual void visit(shared_ptr<CompiledExp> elem) = 0;
			virtual void visit(shared_ptr<CompiledProduct> elem) = 0;
			virtual void visit(shared_ptr<CompiledSum> elem) = 0;
			virtual void visit(shared_ptr<CompiledVariable> var) = 0;
		};
	} // namespace Compiled

} /* namespace AutoDiff */

#endif /* ITAPEVISITOR_H_ */
