#pragma once

#include "ITrigger.h"

#include <vector>
#include <mutex>
#include <condition_variable>

namespace supplementary
{
	class EventTrigger : public virtual ITrigger
	{
	public:
		EventTrigger();
		virtual ~EventTrigger();
		void run(bool notifyAll = true);
	};
}
