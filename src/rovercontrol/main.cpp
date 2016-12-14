#include <iostream>

#include "RosController.h"
#include "maia.h"


RosController* rosController = nullptr;


int main( int argc, char** argv ) {
    std::cout << "RoverControl Starting ..." << std::endl;

    RosController* rosController = new RosController();
    rosController->init(argc, argv, "RoverControl");
    rosController->start();

    Maia maiacore(rosController);

    maiacore.runMAIA();

    std::cout << "RoverControl Finished!" << std::endl;
    return 0;
}

