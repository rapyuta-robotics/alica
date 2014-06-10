/*
 * Task.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TASK_H_
#define TASK_H_

#include "AlicaElement.h"
#include "TaskRepository.h"

namespace alica
{

	/*
	 *
	 */
	class Task : public AlicaElement
	{
	public:
		Task();
		virtual ~Task();
		const string& getDescription() const;
		void setDescription(const string& description);
		const TaskRepository* getTaskRepository() const;
		void setTaskRepository( TaskRepository* taskRepository);

	private:
		string description;
		TaskRepository* taskRepository;
	};

} /* namespace Alica */

#endif /* TASK_H_ */
