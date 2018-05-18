#pragma once
#include <functional>

namespace alica
{

struct TaskRoleStruct
{
  public:
    TaskRoleStruct(int64_t tid, int64_t rid)
        : taskId(tid)
        , roleId(rid)
    {
    }
    int64_t taskId;
    int64_t roleId;
    bool operator==(const TaskRoleStruct& o) const { return taskId == o.taskId && roleId == o.roleId; }
    bool operator!=(const TaskRoleStruct& o) const { return taskId != o.taskId || roleId != o.roleId; }
    bool operator<(const TaskRoleStruct& o) const { return taskId < o.taskId || (taskId == o.taskId && roleId < o.roleId); }
};

} /* namespace alica */
