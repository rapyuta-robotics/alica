/*
 * TaskRepository.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef TASKREPOSITORY_H_
#define TASKREPOSITORY_H_


#include <list>
#include "AlicaElement.h"

namespace alica
{
	class Task;
	/*
	 *
	 */
	class TaskRepository : public AlicaElement
	{
	public:
		TaskRepository();
		virtual ~TaskRepository();
		const string& getFilename() const;
		void setFilename(const string& filename);
		long getDefaultTask() const;
		void setDefaultTask(long defaultTask);
		list<Task*>& getTasks();
		void setTasks(const list<Task*>& tasks);

	protected:
		string filename;
		long defaultTask;
		list<Task*> tasks;
	};

} /* namespace Alica */

#endif /* TASKREPOSITORY_H_ */
