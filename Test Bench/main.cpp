#include "mbed.h"
#include "../src/main.hpp"

Thread remote(osPriorityHigh);

CANMotor chassis1(3,CANHandler::CANBUS_1,M3508);

int maxspeed = 500;

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

        while(1){
            //chassis1.getFeedback(1);
            chassis1.setPosition(myremote.getStickData(LEFTJOYX, 0, maxspeed));
            CANMotor::tick();
        }

}

