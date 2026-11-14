#include "engine/AlicaTimer.h"

#include <algorithm>
#include <chrono>
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
            const auto period = milliseconds(_period);
            const auto inner_loop_duration = milliseconds(std::min(int64_t(500), _period));
            // Deadlines advance by exactly one period, so sleep overshoot does not accumulate and lower the rate.
            auto end_time = steady_clock::now();
            while (_isActive) {
                end_time += period;

                _userCb();

                auto now = steady_clock::now();
                // A callback that overran its period runs again right away, and the schedule restarts from now.
                if (end_time < now) {
                    end_time = now;
                }
                // Sleep in slices of at most 500ms so that stop() does not wait for a long period to end.
                while (now < end_time && _isActive) {
                    std::this_thread::sleep_for(std::min<steady_clock::duration>(inner_loop_duration, end_time - now));
                    now = steady_clock::now();
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
