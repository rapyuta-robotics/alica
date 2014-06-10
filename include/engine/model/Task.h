/*
 * Task.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TASK_H_
#define TASK_H_

using namespace std;

#include <string>
#include <sstream>

#include "AlicaElement.h"
#include "TaskRepository.h"

namespace alica
{

	/*
	 *
	 */
	class TaskRepository;
	class Task : public AlicaElement
	{
	public:
		Task();
		Task(bool defaultTask);
		virtual ~Task();

		const long IDLEID = -1;
		string toString();

		const string& getDesciption() const;
		void setDesciption(const string& desciption);
		const TaskRepository* getTaskrepository() const;
		void setTaskrepository(const TaskRepository* taskrepository);

	protected:
		bool defaultTask;
		string desciption;
		const TaskRepository* taskrepository;
	};

} /* namespace Alica */

#endif /* TASK_H_ */
