#pragma once

#include <condition_variable>
#include <iostream>
#include <unordered_map>
#include <mutex>

namespace essentials
{

class ITrigger
{
public:
    virtual ~ITrigger() = default;
    virtual void run(bool notifyAllThreads) = 0;

    virtual void registerCV(std::condition_variable* condVar)
    {
        std::lock_guard<std::mutex> lock(_cvVecMtx);
        _registeredCVs[condVar] = false;
    }
    bool isNotifyCalled(std::condition_variable* cv)
    {
        std::lock_guard<std::mutex> lock(_cvVecMtx);
        return _registeredCVs.find(cv) != _registeredCVs.end() && _registeredCVs[cv];
    }
    void setNotifyCalled(std::condition_variable* cv, bool called)
    {
        std::lock_guard<std::mutex> lockGuard(_cvVecMtx);
        if (_registeredCVs.find(cv) != _registeredCVs.end()) {
            _registeredCVs[cv] = called;
        }
    }

protected:
    bool isAnyCVRegistered()
    {
        std::lock_guard<std::mutex> lockGuard(_cvVecMtx);
        return !_registeredCVs.empty();
    }
    void notifyOneCV(std::condition_variable* cv, bool notifyAllThreads)
    {
        std::lock_guard<std::mutex> lockGuard(_cvVecMtx);
        if (_registeredCVs.find(cv) != _registeredCVs.end()) {
            _registeredCVs[cv] = true;
            if (notifyAllThreads) {
                cv->notify_all();
            } else {
                cv->notify_one();
            }
        }
    }
    void notifyEveryCV(bool notifyAllThreads)
    {
        std::lock_guard<std::mutex> lockGuard(_cvVecMtx);
        for (auto& pair : _registeredCVs) {
            pair.second = true;
            if (notifyAllThreads) {
                pair.first->notify_all();
            } else {
                pair.first->notify_one();
            }
        }
    }

private:
    std::mutex _cvVecMtx;
    std::unordered_map<std::condition_variable*, bool> _registeredCVs;
};

} /* namespace essentials */
