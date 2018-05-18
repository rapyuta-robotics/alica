/*
 * TaskRoleStruct.h
 *
 *  Created on: Jul 9, 2014
 *      Author: Stefan Jakob
 */

#ifndef TASKROLESTRUCT_H_
#define TASKROLESTRUCT_H_

#include <functional>

using namespace std;
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

namespace std
{
template <>
struct hash<alica::TaskRoleStruct>
{
    typedef alica::TaskRoleStruct argument_type;
    typedef std::size_t value_type;

    value_type operator()(argument_type const& trs) const
    {
        value_type const h1(std::hash<int64_t>()(trs.taskId));
        value_type const h2(std::hash<int64_t>()(trs.roleId));
        return h1 ^ h2;
    }
};
} // namespace std

#endif /* TASKROLESTRUCT_H_ */
