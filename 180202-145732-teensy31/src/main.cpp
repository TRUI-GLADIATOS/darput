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
IntervalTimer myTimer;
char buton = 0x00;

void rutin(){
  digitalWrite(ledPin, LOW);
  imu.Serialget();
  digitalWrite(ledPin, HIGH);
}

void setup() 
{
  pinMode(22, INPUT);
  pinMode(21, INPUT);
  pinMode(20, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  Serial.begin(1000000);
  Serial3.begin(9600);
  imu.mulai(2);
  myTimer.begin(rutin, 150000);
  //Serial1.begin(1000000);
  //Serial1.transmitterEnable(6);
}



void loop() 
{
  //format data ABCXXXXX
  //A untuk 20, B untuk 21, C untuk 22
  uint8_t c, penghitung;
  if (digitalRead(20))
  buton = 'A';
  else if (digitalRead(21))
  buton = 'B';
  else if (digitalRead(22))
  buton = 'C';
  if(Serial.available())    //waiting input from serial
  {
  c = Serial.read();                //get first byte
      //Masuk ke state mengambil isi sensor Kompas
  penghitung = 0;
  SERIAL_KOMPAS.write(0x31);    //initiate with 0x31 to get data from compass
  delay(6);                     //wait to make sure compass completely send data
  Serial.print("K");
  Serial.flush();
  while (SERIAL_KOMPAS.available()){              
    if ((penghitung>=2)&&(penghitung<=4)){
      c = SERIAL_KOMPAS.read();       //only forward data when compass send angle information except decimal point
      Serial.write(c-0x30+offsetAscii);         //forward data and substract with 30 to only send the number plus '0' to send it's ASCII of number
      Serial.flush();
    }else{
      SERIAL_KOMPAS.read();         //if not about angle, discard data
    }
    penghitung++;                   //increase counter
  }
  Serial.flush();                   //flush data to completely send data in USB serial
    //rutinIMU();
  
   Serial.print("X");
   double hx = imu.getX();
   Serial.print(hx);
  // Serial.flush();
  // Serial.flush();
   Serial.print("E");
   Serial.write(buton);
   buton = 0x00;
  }
}