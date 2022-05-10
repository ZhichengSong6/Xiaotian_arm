#include <csignal>
#include "ArmController.h"

bool stop = false;

void handler(int)
{
	std::cout << "will exit..." << std::endl;
	stop = true;
}

int main(){
    signal(SIGINT, handler);

    ArmController armCtrl;
    std::cout << "Successfully initiate Arm controller! " << std::endl;

    while (!stop)
    {
        if (armCtrl._sharedMemory().waitForRobotWithTimeout(2, 0)){
            armCtrl.run();
            armCtrl._sharedMemory().controllerIsDone();
        }
    }

    armCtrl.~ArmController();
    std::cout << "!!! Controller Destruct Finished !!" << std::endl;
    return 0;
}