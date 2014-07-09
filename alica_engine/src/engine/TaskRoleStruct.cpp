/*
 * TaskRoleStruct.cpp
 *
 *  Created on: Jul 9, 2014
 *      Author: Stefan Jakob
 */

#include <engine/TaskRoleStruct.h>

namespace alica
{


	TaskRoleStruct::TaskRoleStruct(long tid, long rid)
	{
		taskId = tid;
		roleId = rid;
	}

	TaskRoleStruct::~TaskRoleStruct()
	{
	}

	bool TaskRoleStruct::equals(TaskRoleStruct* trs)
	{
		return taskId==trs->taskId && roleId==trs->roleId;
	}

} /* namespace alica */
