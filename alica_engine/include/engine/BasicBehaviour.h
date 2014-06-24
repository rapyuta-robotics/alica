/*
 * BasicBehaviour.h
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef BASICBEHAVIOUR_H_
#define BASICBEHAVIOUR_H_

using namespace std;

#include <string>
#include <iostream>
#include <map>

namespace alica
{

	class BasicBehaviour
	{
	public:
		BasicBehaviour();
		BasicBehaviour(string name);
		virtual ~BasicBehaviour();
		virtual void run(void* msg) = 0;
		const string getName() const;
		void setName(string name);
		void start();
		void stop();

	protected:
		string name;
		map<string, string>* parameters;
		int getOwnId();

	private:


	};
} /* namespace alica */

#endif /* BASICBEHAVIOUR_H_ */
