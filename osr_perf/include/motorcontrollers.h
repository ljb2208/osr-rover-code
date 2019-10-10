#ifndef MOTOR_CONTROLLERS_H
#define OSR_NODE_H

#include <ros/ros.h>
#include "../include/roboclaw.h"

using namespace std;
using namespace ros;

class MotorControllers
{
public:
    MotorControllers();
    ~MotorControllers();

    bool connect();
 
    uint16_t getBattery(uint8_t motorID);
    void getTemp(uint16_t* tempVals);
    void getCurrents(int16_t* currVals);
    int32_t getAddrCount();
    void getErrors(uint16_t* errors);
    void getCornerEnc(uint8_t* encVals);
    void cornerToPosition(int32_t tick[]);
    void sendSignedDutyAccel(uint8_t motorID, int32_t speed);

private:
    Roboclaw* rb;

    bool init();
    
    bool checkVersions();
    void killMotors();
    bool errorCheck();

    int32_t accel_max = 655359;
    float accel_rate = 0.5;

    int32_t accel_pos = 0;
    int32_t accel_neg = 0;

    int32_t addr_count = 5;
    int8_t addr[5];
    uint16_t err[5];
    uint32_t accel[10];
    uint32_t qpps[10];

    int8_t enc_min[4];
    int8_t enc_max[4];

    int8_t mids[4];
    int8_t enc[4];



};
#endif
