/*
 * EntryPoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ENTRYPOINT_H_
#define ENTRYPOINT_H_

#include "AlicaElement.h"
#include "Task.h"

namespace alica
{

	class EntryPoint : public AlicaElement
	{
	public:
		EntryPoint();
		virtual ~EntryPoint();
		const Task* getTask() const;
		void setTask(const Task* task);

	protected:
		const Task* task;

	};

} /* namespace Alica */

#endif /* ENTRYPOINT_H_ */
