

# Class Constructor:

```cpp
Motor(int canID, CANHandler::CANBus bus, motorType type = STANDARD, int ratio = 19, int inverted = false)
```

**canID** : The ID of the motor. Most, if not all motors will blink quickly, and counting the blinks will tell you what the ID of the motor is

**CANBus** : An enum of two possible can busses the [Waveshare Can transciever](https://www.amazon.com/SN65HVD230-CAN-Board-Communication-Development/dp/B00KM6XMXO/ref=sr_1_1?crid=PL0JKI6FA69A&keywords=waveshare+can+transceiver&qid=1649575254&sprefix=waveshare+can+transceiv%2Caps%2C323&sr=8-1) could be on: CANBUS_1 or CANBUS_2.

**motorType** : enum that determines what kind of motor is being represented by a specific Motor Object.

**ratio** : measure of what the motor's gear ratio from the motor -> output shaft. Mainly used for Position PID control when the user wants a desired output shaft angle. 

    Ex: The M3508 has a 19:1 gearbox. Therefore this value would be 19.

**inverted** : Boolean value to invert the motor's forward rotation



## Examples:

Some sample motor creations below:  
`Motor gimbalXZ(5, CANBUS_1, GM6020, 1, false);`

`Motor turretYaw(3, CANBUS_2, M3508);`



___

# Motor movement:

Using the PID for the motor class requires you to set up the PID values for the motor, separately for both Position and Speed, using these two functions:  

`void setPositionPID(double Kp, double Ki, double Kd)`

`void setSpeedPID(double Kp, double Ki, double Kd)`

You can cap the integral or cap the output of the PID with these functions:  

`void setPositionIntegralCap(double cap)`

`void setPositionOutputCap(double cap)`

and

`void setSpeedIntegralCap(double cap)`

`void setSpeedOutputCap(double cap)`

Once the PID is set up, you can just give it a speed or a position:

`void setDesiredPos(int value)`

`void setDesiredSpeed(int value)`

or you can just give it a current:

`void setDesiredCurrent(int value)`

Finally, the most important part of using the Motor class is running Motor::tick(); to send all the motor values, get feedback values, and calculate multiTurn angle. 

***

# Requirements to making the Motor class work:

There are some things you need to make sure you have to make the motor class work:

## 1. Attach CANHandler

To make the Motor class work, you need to attach a CANHandler object to it, which stores two can busses that we use to make the motors run.

You do this with this function

`Motor::setCANHandler(&handler);`

Where handler is a CANHandler object.

## 2. Motor::tick()

This is explained more later, but you need to have a

`Motor::tick();`

at the end of your loops, or on a scheduler to send all the motor values, as well as some other things.

IT IS VITAL THAT YOU CALL THIS AT THE END OF EACH LOOP

```cpp
Motor::tick();
```

## **VERY IMPORTANT**

Keep in mind that while the GM6020 motors are technically capable to operate on canIDs 1-7, CAN restrictions do not allow them to operate on IDs other than 5-7. This means with two CAN busses, you have a limit of 6 GM6020s. Motors using the C620s do not have any limitations like this.

___

# Examples

Example code with PID is:

```cpp
#include "mbed.h"
#include "motor.hpp"

CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

int main(){
    motorDebug = 0;

    Motor::setCANHandler(&canPorts);

    Motor standard(1,CANHandler::CANBUS_1,STANDARD);
    Motor gimbly(7,CANHandler::CANBUS_1,GIMBLY);

    gimbly.setPositionPID(5, 0, 10);
    standard.setSpeedPID(0.5, 0, 2);

    gimbly.setPositionIntegralCap(4000);
    standard.setSpeedOutputCap(1000);

    int val = 8738;
    while(1){
        standard.setDesiredSpeed(2000);

        gimbly.setDesiredPos(val);

        Motor::tick();

        printf("Speed:%d\n",standard.getSpeed());
    }
}
```