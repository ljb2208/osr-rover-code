#include "../include/motorcontrollers.h"

MotorControllers::MotorControllers()
{
    //set motor controller addresses
    addr[0] = 128;
    addr[1] = 129;
    addr[2] = 130;
    addr[3] = 131;
    addr[4] = 132;

    accel_pos = int((accel_max /2) + accel_max * accel_rate);
    accel_neg = int((accel_max /2) - accel_max * accel_rate);

    rb = NULL;

    
}

MotorControllers::~MotorControllers()
{
    if (rb != NULL)
        delete rb;
}

bool MotorControllers::connect()
{
    string port = "/dev/ttyAMA0";
    uint32_t baud_rate = 115200;

    rb = new Roboclaw(port, baud_rate);
    // rb = Roboclaw(&port, baud_rate);

    init();

    return true;
}

bool MotorControllers::init()
{
    bool result = false;
    //check versions
    result = checkVersions();

    if (result == false)
        return result;

    ROS_INFO_STREAM("Successfully connected to Roboclaws\r\n");

    killMotors();
   
    int32_t enc_pos = 0;
    for (int i=0; i < addr_count; i++)
    {
        if (addr[i] == 131 || addr[i] == 132)
        {
            float kp, ki, kd;
            uint32_t ki_max, dead_zone, min, max;

            rb->ReadM1PositionPID(addr[i], kp, ki, kd, ki_max, dead_zone, min, max);

            enc_min[enc_pos] = min;
            enc_max[enc_pos] = max;
            enc_pos++;

            rb->ReadM2PositionPID(addr[i],  kp, ki, kd, ki_max, dead_zone, min, max);

            enc_min[enc_pos] = min;
            enc_max[enc_pos] = max;
            enc_pos++;

        }
    }

    for (int i=0; i < addr_count; i++)
    {
        rb->WriteNVM(addr[i]);
    }

    for (int i=0; i < addr_count; i++)
    {
        rb->ReadNVM(addr[i]);
    }

    int32_t counter_pos = 0;
    for (int i=0; i < addr_count; i++)
    {
        float kp, ki, kd; 
        uint32_t qpps_val;

        rb->ReadM1VelocityPID(addr[i], kp, ki, kd, qpps_val); 
        qpps[counter_pos] = qpps_val;
        accel[counter_pos] = qpps[counter_pos] * 2;
        counter_pos++;

        rb->ReadM2VelocityPID(addr[i], kp, ki, kd, qpps_val); 
        qpps[counter_pos] = qpps_val;
        accel[counter_pos] = qpps[counter_pos] * 2;
        counter_pos++;
    }

    if (errorCheck() == true)
        result = false;
    else
        result = true;
    
    killMotors();

    return result;
}

bool MotorControllers::checkVersions()
{
    bool result = true;
    
    for (int i=0; i < addr_count; i++)
    {
        char* ver = new char[48];
        if (rb->ReadVersion(addr[i], ver))
            ROS_INFO_STREAM("Roboclaw address: " << addr[i] << " Version: " << ver << "\r\n");
        else
        {
            result = false;
            ROS_ERROR_STREAM("Roboclaw address: " << addr[i] << " could not retreive version\r\n");
        }

        delete ver;
    }

    return result;
}

void MotorControllers::killMotors()
{
    for (int i=0; i < addr_count; i++)
    {
        rb->ForwardM1(addr[i], 0);
        rb->ForwardM2(addr[i], 0);
    }
}

bool MotorControllers::errorCheck()
{
    bool result = false;
    for (int i=0; i < addr_count; i++)
    {
        bool valid;
        err[i] = rb->ReadError(addr[addr_count], &valid);
    }

    for (int i=0; i < addr_count; i++)
    {
        if (err[i] != 0)
        {
            result = true;
            ROS_ERROR_STREAM("Error in motor controller. Address: " << addr[i] << " Error: " << err[i] << "\r\n");
            killMotors();
        }
    }

    return result;
}

void MotorControllers::getCornerEnc(uint8_t* encVals)
{
    int writeIndex = 0;
    for (int i=0; i < addr_count; i++)
    {
        int index = ceil((i+1)/2.0) + 2;

        uint8_t encVal;
        bool valid;

        if (! (i%2))
            rb->ReadEncM1(addr[index], &encVal, &valid);
        else
            rb->ReadEncM2(addr[index], &encVal, &valid);
        
        encVals[writeIndex] = encVal;
        writeIndex++;
    }
}

// need to check unclear what the logic is
void MotorControllers::cornerToPosition(int32_t tick[])
{

    int32_t speed = 1000;
    int32_t accel = 2000;

    for (int i=0; i < addr_count; i++)
    {
        uint8_t index = ceil((i+1)/2.0)+2;

        if (tick[i] != -1)
        {
            if (i % 2)
                rb->SpeedAccelDeccelPositionM2(addr[index], accel, speed, accel, tick[i], 1);
            else
                rb->SpeedAccelDeccelPositionM1(addr[index], accel, speed, accel, tick[i], 1);
        }
        else
        {
            if (!(i % 2))
                rb->ForwardM1(addr[index], 0);
            else
                rb->ForwardM2(addr[index], 0);
        }
        
    }
}

void MotorControllers::sendSignedDutyAccel(uint8_t motorID, int32_t speed)
{
    uint8_t m_addr = addr[int(motorID / 2)];
    
    int32_t accel_val = accel_neg;
    if (speed > 0)
        accel_val = accel_pos;
    
    if (! (motorID % 2))
        rb->SpeedAccelM1(m_addr, accel_val, speed);
    else
        rb->SpeedAccelM2(m_addr, accel_val, speed);
}

uint16_t MotorControllers::getBattery(uint8_t motorID)
{  
    bool valid = false;
    uint16_t val = rb->ReadMainBatteryVoltage(motorID, &valid);

    if (valid)
        return val;
    else 
        return 0;
}

void MotorControllers::getTemp(uint16_t* tempVals)
{
    for (int i=0; i < addr_count; i++)
    {
        uint16_t val;
        rb->ReadTemp(i, val);
        tempVals[i] = val;
    }
}

void MotorControllers::getCurrents(int16_t* currVals)
{
    for (int i=0; i < addr_count; i++)
    {
        int16_t curr1, curr2;
        rb->ReadCurrents(addr[i], curr1, curr2);

        currVals[i] = curr1;
        currVals[i+1] = curr2;
    }
}

void MotorControllers::getErrors(uint16_t* errorVals)
{
    errorCheck();

    for (int i=0; i < addr_count; i++)
    {
        errorVals[i] = err[i];
    }
}

int32_t MotorControllers::getAddrCount()
{
    return addr_count;
}