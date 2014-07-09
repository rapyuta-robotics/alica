/*
 * TaskRoleStruct.h
 *
 *  Created on: Jul 9, 2014
 *      Author: Stefan Jakob
 */

#ifndef TASKROLESTRUCT_H_
#define TASKROLESTRUCT_H_

using namespace std;

#include <algorithm>

namespace alica
{

	struct TaskRoleStruct
	{
	public:
		TaskRoleStruct(long tid, long rid);
		virtual ~TaskRoleStruct();
		long taskId;
		long roleId;
		bool equals(TaskRoleStruct* trs);
		int getHashCode();
	};

} /* namespace alica */

#endif /* TASKROLESTRUCT_H_ */
