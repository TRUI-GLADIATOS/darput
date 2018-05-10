#include <MPU_OOP.h>
#include <i2c_t3.h>
#include <Arduino.h>

#define RESTRICT_PITCH // Comment out to restrict roll to Â±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define GYRO_WINDOW_SIZE    100
#define ACCEL_WINDOW_SIZE   30
#define MARGIN_OF_SD        2.0

MPU6050::MPU6050(){}

void MPU6050::pakaiKalman(){
    //digitalWrite(13, HIGH);
    double FBaccel = getRLaccel();
    double RLaccel = getFBaccel();

    Kalman kalmanX;
    kalmanX.setAngle(RLaccel);
  
    Kalman kalmanY;
    kalmanY.setAngle(FBaccel);

    double times = millis();
    double dt = times - last_time;


    if (last_time == 0.0)
    {
        dt = 0.0;
    }

    last_time = times;
    double kalmanRLaccel = kalmanX.getAngle(RLaccel, gyroY, dt); // Calculate the angle using a Kalman filter
    double kalmanFBaccel = kalmanY.getAngle(FBaccel, gyroZ, dt); // Calculate the angle using a Kalman filter
    
    if(m_CalibrationStatus == 0 || m_CalibrationStatus == -1)
    {

        if(buf_idx < GYRO_WINDOW_SIZE)
        {
            //if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
            
                // fb_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L);
                // rl_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L);
                fb_gyro_array[buf_idx] = gyroZ;
                rl_gyro_array[buf_idx] = gyroY;
                
                buf_idx++;
                
        }
        else
        {
            double fb_sum = 0.0, rl_sum = 0.0;
            double fb_sd = 0.0, rl_sd = 0.0;
            double fb_diff, rl_diff;
            double fb_mean = 0.0, rl_mean = 0.0;

            buf_idx = 0;
            
            for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
            {
                
                fb_sum += fb_gyro_array[i];
                rl_sum += rl_gyro_array[i];
            }
            fb_mean = fb_sum / GYRO_WINDOW_SIZE;
            rl_mean = rl_sum / GYRO_WINDOW_SIZE;

            fb_sum = 0.0; rl_sum = 0.0;
            for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
            {
                fb_diff = fb_gyro_array[i] - fb_mean;
                rl_diff = rl_gyro_array[i] - rl_mean;
                fb_sum += fb_diff * fb_diff;
                rl_sum += rl_diff * rl_diff;
            }
            fb_sd = sqrt(fb_sum / GYRO_WINDOW_SIZE);
            rl_sd = sqrt(rl_sum / GYRO_WINDOW_SIZE);
            Serial.println(fb_sd);
            Serial.println(rl_sd);
            if(fb_sd < MARGIN_OF_SD && rl_sd < MARGIN_OF_SD)
            {
                m_FBGyroCenter = (int)fb_mean;
                m_RLGyroCenter = (int)rl_mean;
                m_CalibrationStatus = 1;
            }
            else
            {
                m_FBGyroCenter = 512;
                m_RLGyroCenter = 512;
                m_CalibrationStatus = -1;
            }
        }
        digitalWrite(13, LOW);
    }

    if(m_CalibrationStatus == 1)
    {
            // MotionStatus::FB_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) - m_FBGyroCenter;
            // MotionStatus::RL_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) - m_RLGyroCenter;
            // MotionStatus::RL_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L);
            // MotionStatus::FB_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L);

            FB_GYRO = gyroZ - m_FBGyroCenter;
            RL_GYRO = gyroY - m_RLGyroCenter;
            RL_ACCEL = kalmanRLaccel;
            FB_ACCEL = kalmanFBaccel;
			
			
            fb_array[buf_idx] = FB_ACCEL;
            if(++buf_idx >= ACCEL_WINDOW_SIZE) buf_idx = 0;
     
        int sum = 0, avr = 512;
        for(int idx = 0; idx < ACCEL_WINDOW_SIZE; idx++)
            sum += fb_array[idx];
        avr = sum / ACCEL_WINDOW_SIZE;
    digitalWrite(13, HIGH);
       // FB_ACCEL = avr;
    }
    
}

void MPU6050::mulai(uint8_t tipe){
    if (tipe==1)
        Wire.begin(I2C_MASTER, 0x00 , I2C_PINS_16_17, I2C_PULLUP_INT);
    else if(tipe==2)
        Wire.begin(I2C_MASTER, 0x00 , I2C_PINS_18_19, I2C_PULLUP_INT);
    else
        Wire.begin(I2C_MASTER, 0x00 , I2C_PINS_16_17, I2C_PULLUP_INT);

    Wire.beginTransmission(MPU);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Serial.begin(9600);

    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
    accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_Y OUT_H) & 0x3E (ACCEL_YOUT_L)
    accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Wire.read()<<8|Wire.read();
    gyroX = Wire.read()<<8|Wire.read();
    gyroY = Wire.read()<<8|Wire.read();
    gyroZ = Wire.read()<<8|Wire.read();
    /*while ((accX == 0) || (accY == 0) || (accZ == 0)){
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
        accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
        accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_Y OUT_H) & 0x3E (ACCEL_YOUT_L)
        accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Wire.read()<<8|Wire.read();
        gyroX = Wire.read()<<8|Wire.read();
        gyroY= Wire.read()<<8|Wire.read();
        gyroZ = Wire.read()<<8|Wire.read();
    }*/
    pakaiKalman();
}

void MPU6050::Serialget(){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
    accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_Y OUT_H) & 0x3E (ACCEL_YOUT_L)
    accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Wire.read()<<8|Wire.read();
    //Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyroY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyroZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    /*while ((accX == 0) || (accY == 0) || (accZ == 0)){
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
        accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
        accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_Y OUT_H) & 0x3E (ACCEL_YOUT_L)
        accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Wire.read()<<8|Wire.read();
        gyroX = Wire.read()<<8|Wire.read();
        gyroY = Wire.read()<<8|Wire.read();
        gyroZ = Wire.read()<<8|Wire.read();
    }*/
    pakaiKalman();
}

float MPU6050::getFBaccel()
{
	float FBaccel, FBaccelDeg;
	FBaccel = atan((float)accX/sqrt(pow(accZ,2)+pow(accY,2)));
	FBaccelDeg = (FBaccel/1.5708*512)+512;	
	return FBaccelDeg;
}

float MPU6050::getRLaccel()
{
	float RLaccel, RLaccelDeg;
	RLaccel = atan((float)accY/sqrt(pow(accZ,2)+pow(accY,2)));
	RLaccelDeg = (RLaccel/1.5708*512)+512;	
	return RLaccelDeg;
}