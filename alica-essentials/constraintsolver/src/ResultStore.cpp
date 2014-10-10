/*
 * ResultStore.cpp
 *
 *  Created on: Oct 10, 2014
 *      Author: Philipp
 */

#include <ResultStore.h>

namespace alica
{
	namespace reasoner
	{
//		bool ResultStore::lockReady = false;

		ResultStore::ResultStore()
		{
			// TODO Auto-generated constructor stub

		}

		ResultStore::~ResultStore()
		{
			// TODO Auto-generated destructor stub
		}

		shared_ptr<ResultStore> ResultStore::get()
		{
			static shared_ptr<ResultStore> instance;
			return instance;
//			return &instance;
		}

	} /* namespace reasoner */
} /* namespace alica */
