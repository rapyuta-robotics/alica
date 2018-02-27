#pragma once

#include "engine/IAlicaClock.h"
namespace alica_dummy_proxy {

	class AlicaSystemClock : public virtual alica::IAlicaClock
	{
		public:
			AlicaSystemClock();
			virtual ~AlicaSystemClock();
			virtual alica::AlicaTime now();
			virtual void sleep(long us);
	};

} /* namespace alica */
