#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <mutex>
#include <condition_variable>

#include "RosController.h"

class Communication {
public:
    Communication()
        : communicationLock(nullptr)
        , inputAvailable(nullptr)
        , updated(false)
    {}

    virtual ~Communication()
    {
    }

    void setSignal(std::mutex *lock, std::condition_variable *inInputAvailable)
    {
        communicationLock = lock;
        inputAvailable = inInputAvailable;
    }

    bool bUpdated()
    {
        bool current = updated;
        updated = false;
        return current;
    }

protected:
    void signal()
    {
        if (!communicationLock) return;

        std::unique_lock<std::mutex> lock(*communicationLock, std::defer_lock);
        if(inputAvailable && lock.try_lock()) {
            inputAvailable->notify_one();
        }
        updated = true;
    }

    static std::mutex dataChangeLock;

private:
    std::mutex *communicationLock;
    std::condition_variable *inputAvailable;
    bool updated;

};

#endif // COMMUNICATION_H
