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
namespace alica {

struct TaskRoleStruct {
public:
    TaskRoleStruct(long tid, long rid);
    virtual ~TaskRoleStruct();
    long taskId;
    long roleId;
    bool equals(TaskRoleStruct trs);
};

} /* namespace alica */

namespace std {
template <>
struct hash<alica::TaskRoleStruct> {
    typedef alica::TaskRoleStruct argument_type;
    typedef std::size_t value_type;

    value_type operator()(argument_type const& trs) const {
        value_type const h1(std::hash<long>()(trs.taskId));
        value_type const h2(std::hash<long>()(trs.roleId));
        return h1 ^ h2;
    }
};
}  // namespace std

#endif /* TASKROLESTRUCT_H_ */
