#ifndef MPU_OOP_H
#define MPU_OOP_H

#include <kalman.h>
#include <Arduino.h>

class MPU6050{
    
    private:
    Kalman kalmanX; // Create the Kalman instances
    Kalman kalmanY;

    /* IMU Data */
    double accX, accY, accZ;
    double gyroX, gyroY, gyroZ;
    int16_t tempRaw;

    double gyroXangle, gyroYangle; // Angle calculate using the gyro only
    double compAngleX, compAngleY; // Calculated angle using a complementary filter
    double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
    double FBaccel;

    uint32_t timer;
    const int MPU=0x68;  // I2C address of the MPU-6050 

    public:
        MPU6050();
        void mulai(uint8_t tipe); //1 -> 16, 17; 2 -> 18,19
        void Serialget();
        double getX(){
            return FBaccel;
        }
        double getY(){
            return kalAngleY;
        }
};

#endif