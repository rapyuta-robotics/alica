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
            auto inner_loop_duration = milliseconds(std::min(int64_t(500), _period));
            while (_isActive) {
                auto now = system_clock::now().time_since_epoch();
                _userCb();
                auto end_time = now + milliseconds(_period);
                // This second while is a trick to loop more than once, in 500ms intervals,
                // while we wait for _period milliseconds. This allows us to exit this
                // loop earlier without having to wait for the full duration when we Ctrl+C and kill this process.
                while (now < end_time && _isActive) {
                    if (now + inner_loop_duration < end_time) {
                        std::this_thread::sleep_for(inner_loop_duration);
                    } else {
                        std::this_thread::sleep_for(end_time - now);
                    }
                    now = system_clock::now().time_since_epoch();
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
