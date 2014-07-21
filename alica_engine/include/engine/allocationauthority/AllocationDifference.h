/*
 * AllocationDifference.h
 *
 *  Created on: Jul 17, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALLOCATIONDIFFERENCE_H_
#define ALLOCATIONDIFFERENCE_H_

using namespace std;

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

namespace alica
{

	class EntryPointRobotPair;

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
		bool equals(AllocationDifference* other);
		void applyDifference(AllocationDifference* other);
		string toString();

	private:
		vector<EntryPointRobotPair*> additions;
		vector<EntryPointRobotPair*> subtractions;

	protected:
		AllocationDifference::Reason reason;

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
