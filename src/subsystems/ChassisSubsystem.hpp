#include "mbed.h"
#include "../../util/motor/motor.hpp"
#include <cstdlib>
#ifndef chassis_subsystem_cpp
#define chassis_subsystem_cpp

enum chassisMovementMode{TANK, MECANUM, BEYBLADE, RIGHT_ANGLE_MECANUM, SENTRY};

class ChassisSubsystem{
    public:
        Motor motors[4];
        chassisMovementMode chassisMode;

        //       \\\       ///
        //       \0\-------/1/
        //       \\\       ///
        //        |         | 
        //        |         | 
        //        |         | 
        //       \\\       ///
        //       \2\-------/3/
        //       \\\       ///


        ChassisSubsystem(CANHandler::CANBus bus, int motorCanID1_TL, int motorCanID2_TR, int motorCanID3_BL, int motorCANID4_BR, motorType type, chassisMovementMode cMode = MECANUM):
            motors{
                Motor(motorCanID1_TL,bus,type),
                Motor(motorCanID1_TL,bus,type),
                Motor(motorCanID1_TL,bus,type),
                Motor(motorCanID1_TL,bus,type)}
        {
            //do something here
            chassisMode = cMode;
        }

        /**
         * @brief Move the chassis
         * 
         * @param x linear sideways movement (-1,1)
         * @param y forward and backward movement (-1,1)
         * @param rx rotation (-1,1)
         * @param magnitude the speed at which the chassis is moving
         */ 
        void move(int x, int y, int rx, int magnitude){
            if(chassisMode == MECANUM){
                motors[0].setDesiredSpeed(magnitude * (y + x + rx));
                motors[1].setDesiredSpeed(-magnitude * (y - x - rx));
                motors[2].setDesiredSpeed(magnitude * (y - x + rx));
                motors[3].setDesiredSpeed(-magnitude * (y + x - rx));
            }if(chassisMode == TANK){
                motors[0].setDesiredSpeed(magnitude * (y + rx));
                motors[1].setDesiredSpeed(-magnitude * (y - rx));
                motors[2].setDesiredSpeed(magnitude * (y + rx));
                motors[3].setDesiredSpeed(-magnitude * (y - rx));
            }if(chassisMode == SENTRY){
                motors[0].setDesiredSpeed(magnitude * (x));
                motors[1].setDesiredSpeed(magnitude * (x));
            }if(chassisMode == BEYBLADE){
                //How does this work??
            }

        }
        //add methods to make it easy for an end-user to use chassis without worry.
};

#endif