#pragma once

#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>

namespace supplementary
{

class ITrigger
{
  public:
    virtual ~ITrigger() {}
    void registerCV(std::condition_variable* condVar)
    {
        std::lock_guard<std::mutex> lock(cvVec_mtx);
        registeredCVs[condVar] = false;
    }
    virtual void run(bool notifyAll = true) = 0;
    bool isNotifyCalled(std::condition_variable* cv) { return registeredCVs.find(cv) != registeredCVs.end() && registeredCVs[cv]; }
    void setNotifyCalled(bool called, std::condition_variable* cv)
    {
        if (registeredCVs.find(cv) != registeredCVs.end()) {
            registeredCVs[cv] = called;
        }
    }

  protected:
    void notifyAll(bool notifyAll)
    {
        for (auto& pair : registeredCVs) {
            pair.second = true;
            if (notifyAll) {
                pair.first->notify_all();
            } else {
                pair.first->notify_one();
            }
        }
    }
    std::mutex cv_mtx;
    std::mutex cvVec_mtx;
    std::map<std::condition_variable*, bool> registeredCVs;
};

} /* namespace supplementary */
