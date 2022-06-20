#include "../src/main.hpp"
#include <cstdlib>

#define PI 3.14159265

//CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

//DJIRemote myremote(PA_0, PA_1);
//CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

//NewChassisSubsystem chassis(4,2,1,3, CANHandler::CANBUS_1, C620);
int pitchval = 0;
CANMotor LF(4,CANHandler::CANBUS_1,M3508);
CANMotor RF(2,CANHandler::CANBUS_1,M3508);
CANMotor LB(1,CANHandler::CANBUS_1,M3508);
CANMotor RB(3,CANHandler::CANBUS_1,M3508);

CANMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
CANMotor pitch(6, CANHandler::CANBUS_1, GIMBLY);
CANMotor indexer(7, CANHandler::CANBUS_1, C610);

PWMMotor RFLYWHEEL(D12);
PWMMotor LFLYWHEEL(D11);

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);
    yaw.pidPosition.debug = true;
    //pitch.zeroPos();
    yaw.setPositionPID(1.25, 0, 0);

    while (true) {
        
        float WHEELBASE_WIDTH = 0.36;
        float WHEELBASE_LENGTH = 0.41;
        float GIMBAL_X_OFFSET = 0;
        float GIMBAL_Y_OFFSET = 0;
        

        // if (myremote.getSwitchData(RSWITCH) == 2) {
        //     chassis.move(lY,lX,rX);
        // }
        //chassis.move(3000,0,0);
        //chassis.move(lY * 2,lX * 2,rX * 2);
        //chassis.movePow(lY * 4,0,0);
    
        // printf("%d %d %d %d\n",
        //     chassis.getMotor(LEFT_FRONT).getData(VELOCITY),
        //     chassis.getMotor(RIGHT_FRONT).getData(VELOCITY),
        //     chassis.getMotor(LEFT_BACK).getData(VELOCITY),
        //     chassis.getMotor(RIGHT_BACK).getData(VELOCITY));
        // LF.setPower(lY);
        // RF.setPower(lY);
        // LB.setPower(lY);
        // RB.setPower(lY);
        // double jAngle = std::atan((double)(lY)/lX);
        // if(lX < 0 != lY < 0)
        //     jAngle = PI/2 + jAngle;
        double jAngle = std::atan2(lY,lX);
        double tAngle = std::fmod((PI * 2 - 5.53 + (PI * yaw.getData(ANGLE)/ 4096)),(PI * 2));
        if(lY < 0)
            jAngle = jAngle + PI * 2;
        //double angle = jAngle - tAngle;
        double angle = yaw.getData(ANGLE) / 8192.0 * PI * 2;
        angle -= PI/4;
        




        if(rS == 1){
            angle = jAngle - tAngle;

            // if (pitchval < 1500) // lowerbound
            //     pitchval = 1500;
            // if (pitchval > 3000) //upperbound
            //     pitchval = 3000;
            // pitchval += (int)myremote.getStickData(RIGHTJOYY, 0, 3);
            
            // printf("%d\n", pitchval);

            int LFa = lY + lX + Wh, RFa = lY - lX - Wh, LBa = lY - lX + Wh, RBa = lY + lX - Wh;
            LF.setSpeed(LFa);
            RF.setSpeed(-RFa);
            LB.setSpeed(LBa);
            RB.setSpeed(-RBa);
            //yaw.setPosition(yaw.getData(MULTI) - (rX * 2 ));
            //yaw.setPosition(rX * 4 + 4000);
            yaw.setPower(-rX * 4);
            pitch.setPosition(2250 + rY * 3/4);


            

            //pitch.printAllMotorData();
            //printf("%d\n", pitch.getData(ANGLE));
            CANMotor::printChunk(CANHandler::CANBUS_1,2);
        }else if(rS == 2){
            LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
            yaw.setPower(0);
            yaw.multiTurn = 0;
            remotePrint();
        }else if(rS == 3){
            int speedMult = 2;

            float raw_x = lX * speedMult;
            float raw_y = lY * speedMult;

            // change of angle from the original position
            //radian = sig->gimbal.yaw_ecd / ECD_PERIOD * 2 * PI;

            // apply rotational matrix
            float x = (float) (raw_x * cos(angle) - raw_y * sin(angle));
            float y = (float) (raw_x * sin(angle) + raw_y * cos(angle));

            int beyblade_rotation = 135;
            // LF.setSpeed(x + y + beyblade_rotation);
            // RF.setSpeed(x - y + beyblade_rotation);
            // LB.setSpeed(-x + y + beyblade_rotation);
            // RB.setSpeed(-x - y + beyblade_rotation);
            int compensation = 200;
            //yaw.setPosition(- yaw.getData(MULTI) + (rY * 2 + compensation));
            yaw.setPosition(-rX * 8);
            //yaw.setSpeed(20);
            //printf("nu\n");

            printf("MLTI:%d\t\tTIME_SINCE:%lu\n",yaw.getData(MULTI),yaw.timeSinceLastFeedback);
        }
        int indexJamTime = 0;
        if(lS == 2){
            indexer.setPower(0);
            LFLYWHEEL.set(0);
            RFLYWHEEL.set(0);
        }else if(lS == 3){
            if(abs(indexer.getData(TORQUE)) > 1000 & abs(indexer.getData(VELOCITY)) < 20){ //jam
                indexJamTime = us_ticker_read() /1000;
            }
            if(us_ticker_read() / 1000 - indexJamTime < 500){
                indexer.setPower(-7000); //jam
                printf("JAMMMMM- ");
            }else if(us_ticker_read() / 1000 - indexJamTime < 750){
                indexer.setPower(7000); //jam
                printf("POWER FORWARD- ");
            }else{
                indexer.setSpeed(-700);
            }
            printf("AUTO-PWR:%d Jam-Free:%dms TORQ:%d, VELO:%d\n",indexer.powerOut,us_ticker_read() / 1000 - indexJamTime, indexer.getData(TORQUE), indexer.getData(VELOCITY));
            LFLYWHEEL.set(60);
            RFLYWHEEL.set(60);
        }else if(lS == 1){
            indexer.setPower(rY * 3);
            //CANMotor::printChunk(CANHandler::CANBUS_1,1);
            //printf("MANUAL-PWR:%d VELO:%d\n", indexer.powerOut, indexer.getData(VELOCITY));
            LFLYWHEEL.set(40);
            RFLYWHEEL.set(40);
        }

        // if(lS == 2)
        //     yaw.setSpeed(rX/10);
        // else if(lS == 1)
        //     yaw.setPower(0);
        //printf("RAD:%d ANGL:%d YAW:%d ATAN(%d):%d\n", int(1000 * tAngle), int(angle * 1000), yaw.getData(ANGLE), int((double)(lY)/lX * 1000), int(std::atan2(lY,lX) * 1000));
        //CANMotor::printChunk(CANHandler::CANBUS_1,0);
        //printf("jAngle:%d\n",int(jAngle * 1000));


        // printf("%d %d %d %d\n",
        //     LF.getData(VELOCITY),
        //     RF.getData(VELOCITY),
        //     LB.getData(VELOCITY),
        //     RB.getData(VELOCITY));

        

        //for(int i = 0; i < 12; i)

        // else {
        //     //yawval+= (int)myremote.getStickData(LEFTJOYY, 0, maxpitchyawchange);
        //     pitchval+=(int)myremote.getStickData(LEFTJOYY, 0, maxpitchyawchange);

        //     if (pitchval > 240)
        //         pitchval = 240;
        //     if (pitchval < 180)
        //         pitchval = 180;
            
        // }
        
        // if (lS == 1) {
        //     LFLYWHEEL.set(revval);
        //     RFLYWHEEL.set(revval);
        //     serializer.setPower(-70);
        // }
        // else if (lS == 2) {
        //     serializer.setPower(90);
        // }
        // else if (lS == 3) {
        //     LFLYWHEEL.set(neutralval);
        //     RFLYWHEEL.set(neutralval);
        //     serializer.setPower(0);
        // }

        //yaw.setDesiredPos(yawval);
        //pitch.setPower(220);

        //CANMotor::tick(lS == 2, rS == 2);
        ThisThread::sleep_for(1ms);
    }
}

