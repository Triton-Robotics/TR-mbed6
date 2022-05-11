#include "mbed.h"
#include "../util/motor/motor.hpp"
#include "../util/communications/include/DJIRemote.hpp"
#include "../util/communications/canHandler.hpp"
#include "../util/algorithms/pid.hpp"
#include "subsystems/ChassisSubsystem.hpp"
#include "subsystems/TurretSubsystem.hpp"
#include "../util/communications/djiremoteuart.hpp"
#include "../util/helperFunctions.hpp"
#include "../util/communications/SerialCommunication.hpp"
//#include "robots/include/infantry.hpp"
static DJIRemote myremote(PA_0, PA_1);

static int lX = 0;
static int lY = 0;
static int rX = 0;
static int rY = 0;
static int Wh = 0;
static int lS = 0;
static int rS = 0;

enum robotType{TEST_BENCH, INFANTRY, HERO, ENGINEER, SENTRY};

static void remoteThread(){
    while(1){
        myremote.remoteUpdate();
        lX = myremote.getStickData(LEFTJOYX,0,1000);
        lY = myremote.getStickData(LEFTJOYY,0,1000);
        rX = myremote.getStickData(RIGHTJOYX,0,1000);
        rY = myremote.getStickData(RIGHTJOYY,0,1000);
        Wh = myremote.getStickData(WHEEL,0,1000);
        lS = myremote.getSwitchData(LSWITCH);
        rS = myremote.getSwitchData(RSWITCH);
        ThisThread::sleep_for(1ms);
    }
}