#include "engine/AlicaTimer.h"

#include <thread>

namespace alica
{

class AlicaSystemTimer::AlicaSystemTimerImpl
{
public:
    AlicaSystemTimerImpl(TimerCb&& userCb, alica::AlicaTime period)
            : _userCb(std::move(userCb))
            , _period(period.inMilliseconds())
            , _isActive(false)
    {
    }

    ~AlicaSystemTimerImpl()
    {
        if (_isActive) {
            stop();
        }
    }

    void start()
    {
        if (_isActive) {
            stop();
        }
        _isActive = true;
        _thread = std::thread([this]() {
            using namespace std::chrono;
            while (_isActive) {
                auto hasRepeatedInternalLoop = false;
                auto startOuterLoop = system_clock::now();
                _userCb();
                int64_t sleep_duration = _period;
                system_clock::time_point startInnerLoop;
                // This second while is a trick to loop more than once, in 500ms intervals,
                // while we wait for sleep_duration milliseconds. This allows us to exit this
                // loop earlier without having to wait for the full duration when we Ctrl+C and kill this process.
                while ((sleep_duration > 0) && _isActive) {
                    auto loop_duration = duration_cast<milliseconds>((system_clock::now() - (hasRepeatedInternalLoop ? startInnerLoop : startOuterLoop)));
                    std::this_thread::sleep_for(milliseconds(std::min(int64_t(500), _period)) - loop_duration);
                    sleep_duration -= std::max(loop_duration.count(), std::min(int64_t(500), _period));
                    startInnerLoop = system_clock::now();
                    hasRepeatedInternalLoop = true;
                }
            }
        });
    }

    void stop()
    {
        _isActive = false;
        if (_thread.joinable()) {
            _thread.join();
        }
    }

    TimerCb _userCb;
    int64_t _period;
    std::thread _thread;
    std::atomic<bool> _isActive;
};

AlicaSystemTimer::AlicaSystemTimer(TimerCb&& userCb, alica::AlicaTime period)
        : _impl(std::make_unique<AlicaSystemTimerImpl>(std::move(userCb), period))
{
    _impl->start();
}

AlicaSystemTimer::~AlicaSystemTimer()
{
    _impl->stop();
}

} // namespace alica
