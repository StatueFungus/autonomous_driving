#ifndef MAIA_H
#define MAIA_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>

#include "rovercontroller.h"
#include "phwrap.h"

class Maia
{
public:
    Maia(RosController* ros);
    ~Maia();

    void runMAIA();

private:
    std::mutex communicationLock_;
    std::condition_variable inputAvailable_;

    //Communication Objects
    Flightcontroller *flightcontroller_;
    Controller *controller_;
};

#endif // MAIA_H
