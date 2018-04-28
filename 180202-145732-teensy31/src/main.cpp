#include <Arduino.h>
//#include <mpu6050.h>
#include <i2c_t3.h>
#include <kalman.h>

#define UART_TXRTSE (2)
#define UART_TXRTSPOL (4)
#define SERIAL_PORT Serial2         //PORT TO SERVO
#define SERIAL_KOMPAS Serial3       

#define offsetAscii '0'
#define ledPin 13
#define RESTRICT_PITCH // Comment out to restrict roll to Â±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
//MPU6050 imu;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
const int MPU=0x68;  // I2C address of the MPU-6050


void rutinIMU();

void setup() 
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  //rev 1.0
  /*
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_INT);
  Serial.begin(1000000);
  SERIAL_PORT.begin(1000000);
  SERIAL_PORT.transmitterEnable(6);
  SERIAL_KOMPAS.begin(9600);

  if (!imu.begin(AFS_2G, GFS_250DPS)) {
        Serial.println("MPU6050 is online...");
    }
    else {
        Serial.println("Failed to init MPU6050");
        while (true) 
            ;
    }
  */
 //rev 2.0
 //prepare for MPU data
  Serial.begin(1000000);
  SERIAL_PORT.begin(1000000);
  SERIAL_PORT.transmitterEnable(6);
  SERIAL_KOMPAS.begin(9600);
  Wire.begin(I2C_MASTER, 0x00 , I2C_PINS_16_17, I2C_PULLUP_INT);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_Y OUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  // end of preparing

  /* 
  // set receiver buffer threshold for interrupt back to 1.
  uint8_t c2 = UART0_C2;
  UART0_C2 = c2 & ~UART_C2_RE; // disable C2[RE] (receiver enable)
  UART0_RWFIFO = 1;            // set receiver threshold
  UART0_C2 = c2;               // restore C2[RE]

  // enable PIN 6 as hardware transmitter RTS with active HIGH.
  CORE_PIN6_CONFIG = PORT_PCR_MUX(3);
  UART0_MODEM = UART_TXRTSE | UART_TXRTSPOL; 
  */
}

void loop() 
{
  
  uint8_t c;
  uint8_t panjang_paket;
  uint8_t penghitung;
  //uint8_t offsetAscii = '0';
  while (!(Serial.available()));    //waiting input from serial
  c = Serial.read();                //get first byte
  if (c==0xFF){                     //check if first serial bytes indicate servo, go to this state
  digitalWrite(ledPin, LOW);
    penghitung = 0;                 //counter as a flag
    delay(3);                       //delay to wait for serial to completely receive data
    SERIAL_PORT.write(c);           //write first byte to servo
    penghitung++;                   //increase counter to show the position relative to byte contain length
    while (penghitung<=3){          //check if all input serial before byte length
      if (Serial.available()){  
        c = Serial.read();          //read serial input from usb
        SERIAL_PORT.write(c);       //forward to servo
        penghitung++;               //increase counter to indicate next position
      }
    }
    panjang_paket = c;            //save length in memory
    uint8_t i=0;                  //counter to count
    while (i<panjang_paket){      //until panjang_paket
      if (Serial.available()){    
        c = Serial.read();
        SERIAL_PORT.write(c);
        i++;
      }
    }
    //digitalWrite(ledPin,HIGH);
    //SERIAL_PORT.flush();          //make sure all buffer in servo line is sent
    delay(2);                     //make sure all data from servo is arrived
    while (SERIAL_PORT.available()){      //check if any left data in servo line serial
      c = SERIAL_PORT.read();     //forward byte from servo
      Serial.write(c);            //to miniPC
    }
    //Serial.flush();               //Make sure data is completely sent to USB
    digitalWrite(ledPin, HIGH);
  }else if(c=='K'){
      //Masuk ke state mengambil isi sensor Kompas
    penghitung = 0;
    SERIAL_KOMPAS.write(0x31);    //initiate with 0x31 to get data from compass
    delay(6);                     //wait to make sure compass completely send data
    while (SERIAL_KOMPAS.available()){              
      if ((penghitung>=2)&&(penghitung<=4)){
        c = SERIAL_KOMPAS.read();       //only forward data when compass send angle information except decimal point
        Serial.write(c-0x30+offsetAscii);         //forward data and substract with 30 to only send the number plus '0' to send it's ASCII of number
      }else{
        SERIAL_KOMPAS.read();         //if not about angle, discard data
      }
      penghitung++;                   //increase counter
    }
    Serial.flush();                   //flush data to completely send data in USB serial
    rutinIMU();
  }
  
}

void rutinIMU(){
  /*
  int16_t ax, ay, az, gx, gy, gz;
  if (imu.getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz)) {
        
        Serial.print("AX");
        Serial.print(ax);
        Serial.print("AY");
        Serial.print(ay);
        Serial.print("AZ");
        Serial.print(az);
        Serial.print("GX");
        Serial.print(gx);
        Serial.print("GY");
        Serial.print(gy);
        Serial.print("GZ");
        Serial.print(gz);
        Serial.println();
        Serial.flush();
    }
  */
 double Tmp; 
 Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_Y OUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
 
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG ;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG ;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s


#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
 
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

 /* Serial.print("AcX = "); Serial.print(accX);
  Serial.print(" | AcY = "); Serial.print(accY);
  Serial.print(" | AcZ = "); Serial.print(accZ);
  //Serial.print(" | Tmp = "); Serial.print(tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(gyroX);
  Serial.print(" | GyY = "); Serial.print(gyroY);
  Serial.print(" | GyZ = "); Serial.println(gyroZ);
  delay(333);*/
 
  /*Serial.print("\n");
  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");*/
  Serial.print("GX"); Serial.print(kalAngleX); 

  
  /*Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");*/
  Serial.print("GY"); Serial.print(kalAngleY); 
  double FBaccel = atan((float)accX/sqrt(pow(accZ,2)+pow(accY,2))); FBaccel = (FBaccel/1.5708*512)+512;	
  Serial.print("FB"); Serial.print(FBaccel);
  double RLaccel = atan((float)accY/sqrt(pow(accZ,2)+pow(accY,2))); RLaccel = (RLaccel/1.5708*512)+512;	
  Serial.print("RL"); Serial.print(RLaccel);
  delay(100);
}
