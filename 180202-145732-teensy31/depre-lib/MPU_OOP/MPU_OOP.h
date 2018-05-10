#ifndef MPU_OOP_H
#define MPU_OOP_H
#define GYRO_WINDOW_SIZE    100
#define ACCEL_WINDOW_SIZE   30

#include <kalman.h>
#include <Arduino.h>

class MPU6050{
    
    private:

    /* IMU Data */
    double accX, accY, accZ;
    double gyroX, gyroY, gyroZ;
    int16_t tempRaw;
    double gyroXangle, gyroYangle; // Angle calculate using the gyro only
    double compAngleX, compAngleY; // Calculated angle using a complementary filter
    double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
    int fb_array[ACCEL_WINDOW_SIZE] = {512,};
    int buf_idx = 0;
    int fb_gyro_array[GYRO_WINDOW_SIZE] = {512,};
    int rl_gyro_array[GYRO_WINDOW_SIZE] = {512,};
    uint32_t timer;
    const int MPU=0x68;  // I2C address of the MPU-6050 

    //vegege
    int m_FBGyroCenter = 512;
	int m_RLGyroCenter = 512;
	
    float getFBaccel();
    float getRLaccel();
    void pakaiKalman();
    double last_time = 0.0;
    int FB_GYRO, RL_GYRO,FB_ACCEL, RL_ACCEL;

    //endofgg
    public:
        MPU6050();
        void mulai(uint8_t tipe); //1 -> 16, 17; 2 -> 18,19
        void Serialget();
        int getX(){
            return FB_ACCEL/100;
        }
        int getY(){
            return RL_ACCEL/10000;
        }
        int m_CalibrationStatus = 0;    
};

#endif