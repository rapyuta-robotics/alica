/*
 * AllocationDifference.h
 *
 *  Created on: Jul 17, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALLOCATIONDIFFERENCE_H_
#define ALLOCATIONDIFFERENCE_H_


#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include <algorithm>

#include "EntryPointRobotPair.h"

namespace alica
{


    /**
     * A representation of the difference between two allocations
     */
    class AllocationDifference final
    {
    public:
        AllocationDifference();
        ~AllocationDifference();
        enum Reason {message, utility, empty};
        AllocationDifference::Reason getReason() const;
        void setReason(AllocationDifference::Reason reason);

        /**
        * Returns whether the difference is empty, i.e., the corresponding allocations are the same
        */
        bool isEmpty() const;

        /**
        * Reset this difference to the empty difference
        */
        void reset();

        /**
        * Apply another difference to this one resulting in the composition of both
        * @param other the AllocationDifference to apply.
        */
        void applyDifference(const AllocationDifference& other);
        std::string toString() const;

        const std::vector<EntryPointRobotPair>& getAdditions() const;
        std::vector<EntryPointRobotPair>& editAdditions();

        void setAdditions(const vector<EntryPointRobotPair>& additions);
        const std::vector<EntryPointRobotPair>& getSubtractions() const;
        std::vector<EntryPointRobotPair>& editSubtractions();
        void setSubtractions(const vector<EntryPointRobotPair>& subtractions);

    private:
        std::vector<EntryPointRobotPair> _additions;
        std::vector<EntryPointRobotPair> _subtractions;
        /**
         * Denoting the reason for an allocation switch
         */
        AllocationDifference::Reason _reason;

    };

} /* namespace alica */


//TODO Get rid of this, toString was meant to be a debug facility.
namespace std
{
    template<>
    struct hash<alica::AllocationDifference>
    {
        typedef alica::AllocationDifference argument_type;
        typedef std::size_t value_type;

        value_type operator()(argument_type & ad) const
        {
            value_type const h1 ( std::hash<std::string>()(ad.toString()) );
            return h1;
        }
    };
}

#endif /* ALLOCATIONDIFFERENCE_H_ */
