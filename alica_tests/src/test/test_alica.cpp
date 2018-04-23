#include <test_alica.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanBase.h>

void step(alica::AlicaEngine* ae) {
    ae->stepNotify();
    do {
        ae->getAlicaClock()->sleep(alica::AlicaTime::milliseconds(33));
    } while (!ae->getPlanBase()->isWaiting());
}