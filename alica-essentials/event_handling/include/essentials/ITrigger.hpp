#pragma once

#include <condition_variable>
#include <iostream>
#include <map>
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
        std::lock_guard<std::mutex> lock(cvVec_mtx);
        registeredCVs[condVar] = false;
    }
    bool isNotifyCalled(std::condition_variable* cv)
    {
        std::lock_guard<std::mutex> lock(cvVec_mtx);
        return registeredCVs.find(cv) != registeredCVs.end() && registeredCVs[cv];
    }
    void setNotifyCalled(std::condition_variable* cv, bool called)
    {
        std::lock_guard<std::mutex> lockGuard(cvVec_mtx);
        if (registeredCVs.find(cv) != registeredCVs.end()) {
            registeredCVs[cv] = called;
        }
    }

protected:
    bool isAnyCVRegistered()
    {
        std::lock_guard<std::mutex> lockGuard(cvVec_mtx);
        return !registeredCVs.empty();
    }
    void notifyOneCV(std::condition_variable* cv, bool notifyAllThreads)
    {
        std::lock_guard<std::mutex> lockGuard(cvVec_mtx);
        if (registeredCVs.find(cv) != registeredCVs.end()) {
            registeredCVs[cv] = true;
            if (notifyAllThreads) {
                cv->notify_all();
            } else {
                cv->notify_one();
            }
        }
    }
    void notifyEveryCV(bool notifyAllThreads)
    {
        std::lock_guard<std::mutex> lockGuard(cvVec_mtx);
        for (auto& pair : registeredCVs) {
            pair.second = true;
            if (notifyAllThreads) {
                pair.first->notify_all();
            } else {
                pair.first->notify_one();
            }
        }
    }

private:
    std::mutex cvVec_mtx;
    std::map<std::condition_variable*, bool> registeredCVs;
};

} /* namespace essentials */
