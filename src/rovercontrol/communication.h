#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <mutex>
#include <condition_variable>

#include "RosController.h"

class Communication {
public:
    Communication()
        : communicationLock_(nullptr)
        , inputAvailable_(nullptr)
        , updated_(false)
    {}

    virtual ~Communication()
    {
    }

    void setSignal(std::mutex *lock, std::condition_variable *inputAvailable)
    {
        communicationLock_ = lock;
        inputAvailable_ = inputAvailable;
    }

    bool updated()
    {
        bool current = updated_;
        updated_ = false;
        return current;
    }

protected:
    void signal()
    {
        if (!communicationLock_) return;

        std::unique_lock<std::mutex> lock(*communicationLock_, std::defer_lock);
        if(inputAvailable_ && lock.try_lock()) {
            inputAvailable_->notify_one();
        }
        updated_ = true;
    }

    static std::mutex dataChangeLock_;

private:
    std::mutex *communicationLock_;
    std::condition_variable *inputAvailable_;
    bool updated_;

};

#endif // COMMUNICATION_H
