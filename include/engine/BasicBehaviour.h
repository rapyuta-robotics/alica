/*
 * BasicBehaviour.h
 *
 *  Created on: Jun 4, 2014
 *      Author: stefan
 */

#ifndef BASICBEHAVIOUR_H_
#define BASICBEHAVIOUR_H_

using namespace std;

#include <string>
#include <map>

namespace alica
{

	class BasicBehaviour
	{
	public:
		BasicBehaviour(string name);
		virtual ~BasicBehaviour();
		virtual void run(void* msg) = 0;
		const string getName() const;
		void setName(string name);
	protected:
		string name;
		map<string, string>* parameters;

		int getOwnId();
	};

} /* namespace alica */

#endif /* BASICBEHAVIOUR_H_ */
