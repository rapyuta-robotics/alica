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

using namespace std;
namespace alica
{

	class EntryPointRobotPair;

	/**
	 * A representation of the difference between two allocations
	 */
	class AllocationDifference
	{
	public:
		AllocationDifference();
		virtual ~AllocationDifference();
		enum Reason {message, utility, empty};
		AllocationDifference::Reason getReason();
		void setReason(AllocationDifference::Reason reason);
		bool isEmpty();
		void reset();
		void applyDifference(AllocationDifference* other);
		string toString();
		vector<shared_ptr<EntryPointRobotPair>>& getAdditions();
		void setAdditions(vector<shared_ptr<EntryPointRobotPair>> additions);
		vector<shared_ptr<EntryPointRobotPair>>& getSubtractions();
		void setSubtractions(vector<shared_ptr<EntryPointRobotPair>> subtractions);

	protected:
		/**
		 * Denoting the reason for an allocation switch
		 */
		AllocationDifference::Reason reason;
		vector<shared_ptr<EntryPointRobotPair>> additions;
		vector<shared_ptr<EntryPointRobotPair>> subtractions;

	};

} /* namespace alica */


//TODO this about other hash calculation
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
