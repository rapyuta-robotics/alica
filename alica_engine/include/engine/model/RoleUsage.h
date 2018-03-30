/*
 * RoleUsage.h
 *
 *  Created on: Jun 10, 2014
 *      Author: Stefan Jakob
 */

#ifndef ROLEUSAGE_H_
#define ROLEUSAGE_H_

namespace alica {
class Role;

/**
 * RoleUsage contains a role with it's priority.
 */
class RoleUsage {
public:
    RoleUsage(int priorityOrder, Role* role);
    virtual ~RoleUsage();
    bool isUsed() const;
    void setUsed(bool used);
    int getPriorityOrder() const;
    void setPriorityOrder(int priorityOrder);
    Role* getRole() const;
    void setRole(Role* role);

protected:
    int priorityOrder;
    Role* role;
    bool used;
};

} /* namespace alica */

#endif /* ROLEUSAGE_H_ */
