#include <Arduino.h>
#include <MPU_OOP.h>

#define UART_TXRTSE (2)
#define UART_TXRTSPOL (4)
#define SERIAL_KOMPAS Serial3       
//#define SERIAL_PORT Serial1
#define offsetAscii '0'
#define ledPin 13

//void rutinIMU();
MPU6050 imu;
//IntervalTimer myTimer;

void rutin(){
  digitalWrite(ledPin, LOW);
  
  digitalWrite(ledPin, HIGH);
}

void setup() 
{
  pinMode(ledPin, OUTPUT);
  
  Serial.begin(1000000);
  imu.mulai(2);
  
  while(imu.m_CalibrationStatus != 1){
    imu.Serialget();
  }
  digitalWrite(ledPin, HIGH);
  //imu.m_CalibrationStatus = 1;
  //myTimer.begin(rutin, 150000);
  //Serial1.begin(1000000);
  //Serial1.transmitterEnable(6);
}



void loop() 
{
  uint8_t c, penghitung;

  while (!(Serial.available()));    //waiting input from serial
  c = Serial.read();                //get first byte
      //Masuk ke state mengambil isi sensor Kompas
  penghitung = 0;
  SERIAL_KOMPAS.write(0x31);    //initiate with 0x31 to get data from compass
  delay(6);                     //wait to make sure compass completely send data
  Serial.print("K");
  //Serial.flush();
  while (SERIAL_KOMPAS.available()){              
    if ((penghitung>=2)&&(penghitung<=4)){
      c = SERIAL_KOMPAS.read();       //only forward data when compass send angle information except decimal point
      Serial.write(c-0x30+offsetAscii);         //forward data and substract with 30 to only send the number plus '0' to send it's ASCII of number
      //Serial.flush();
    }else{
      SERIAL_KOMPAS.read();         //if not about angle, discard data
    }
    penghitung++;                   //increase counter
  }
  //Serial.flush();                   //flush data to completely send data in USB serial
    //rutinIMU();
    imu.Serialget();
   Serial.print("X");
   int hx = imu.getX();
   Serial.print(hx);
   //Serial.flush();
   Serial.print("Y");
   int hy = imu.getY();
   Serial.print(hy);
   //Serial.flush();
   Serial.print("E");

}