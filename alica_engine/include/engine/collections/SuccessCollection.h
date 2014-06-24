/*
 * SuccessCollection.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Stefan Jakob
 */

#ifndef SUCCESSCOLLECTION_H_
#define SUCCESSCOLLECTION_H_

using namespace std;

#include <list>

namespace alica
{
	class EntryPoint;
	class Plan;

	class SuccessCollection
	{
	public:
		SuccessCollection(Plan* plan);
		virtual ~SuccessCollection();
		int getCount() const;
		void setCount(int count);
		EntryPoint** getKeys();
		list<int>** getValues();
		void setSuccess(int robot, EntryPoint* ep);
		void clear();
		list<int>** getRobots();
		void setRobots(list<int>** robots);

	private:

	protected:
		EntryPoint** keys;
		list<int>** values;
		list<int>** robots;
		int count = 0;
	};

} /* namespace alica */

#endif /* SUCCESSCOLLECTION_H_ */
