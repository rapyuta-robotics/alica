/*
 * ResultStore.h
 *
 *  Created on: Oct 10, 2014
 *      Author: Philipp
 */

#ifndef RESULTSTORE_H_
#define RESULTSTORE_H_

#include <memory>

using namespace std;

namespace alica
{
	namespace reasoner
	{

		class ResultStore : public enable_shared_from_this<ResultStore>
		{
		public:
			ResultStore();
			virtual ~ResultStore();

			static shared_ptr<ResultStore> get();

		private:
//			static shared_ptr<ResultStore> instance;
//			static shared_ptr<object> lockObj;
//			static bool lockReady;
		};

	} /* namespace reasoner */
} /* namespace alica */

#endif /* RESULTSTORE_H_ */
