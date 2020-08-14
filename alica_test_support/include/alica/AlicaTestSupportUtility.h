#pragma once

namespace alica {
    class AlicaEngine;
    class AlicaContext;
    class AlicaTestSupportUtility {
    public:
        static alica::AlicaEngine* getEngine(alica::AlicaContext* ac);
    private:
        AlicaTestSupportUtility() = delete;
    };
}
