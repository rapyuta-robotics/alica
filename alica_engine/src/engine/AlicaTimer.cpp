#include "engine/AlicaTimer.h"

namespace alica
{

class AlicaSystemTimer::AlicaSystemTimerImpl
{
public:
    using TimerCb = std::function<void()>;

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
            while (_isActive) {
                _userCb();
                int64_t sleep_duration = _period;
                while ((sleep_duration > 0) && _isActive) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(std::min(int64_t(500), _period)));
                    sleep_duration -= std::min(int64_t(500), _period);
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
        : _impl(std::make_shared<AlicaSystemTimerImpl>(std::move(userCb), period))
{
    _impl->start();
}

AlicaSystemTimer::~AlicaSystemTimer()
{
    _impl->stop();
}

} // namespace alica
