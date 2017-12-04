#pragma once

#include <supplementary/InfoBuffer.h>

namespace alica{
	class AlicaEngine;
}

namespace supplementary{
	class SystemConfig;

	class WorldModel {
	public:
    	WorldModel(); /* <-- Attention: Derived World Models should implement the singleton pattern */
    	virtual ~WorldModel();
		supplementary::InfoTime getTime();
	    bool isMaySendMessages() const;
	    void setMaySendMessages(bool maySendMessages);
	    int getOwnId();
	    bool setEngine(alica::AlicaEngine *ae);
	    alica::AlicaEngine* getEngine();

	protected:
		SystemConfig* sc;
		alica::AlicaEngine* alicaEngine;
		bool maySendMessages;
		int ownID;
	};
}
