#define DXL_BUS_SERIAL1 1
#define DXL_BUS_SERIAL2 2
#define DXL_BUS_SERIAL3 3

#include <math.h>
#include <string.h>

#define PHI (float)57.295779513082320876798154814105
#define SPEED 1023
#define P_GOAL_POSITION    2048
#define P_GOAL_SPEED    15

//=====================================
Dynamixel Dxl(DXL_BUS_SERIAL3);
Dynamixel Dxl1(DXL_BUS_SERIAL3);

byte incomingByte;
byte arah;

float angleRight[6];
float angleLeft[6];
float handRight[5];
float handLeft[5];
float body[4];

float right1 = 93;
float right2 = 93;
float left1  =  93; //dilihat dari hasil waterpass
float left2  =  93; //

int l;


boolean param = true;
int delayTime = 100;
int delayTime2 = 1500;
int delayTime3 = 750;
int normalFootHeight = 190;
int walkDistance = 15;
int walkDistanceOffset = -15;
int tiltOffset = 15;
int walkTilt = 30;
int walkFootLift = normalFootHeight - 55;
int walkAngle = 0;// - kiri, + kanan
int bodyLeanForward = 2; //Variabel untuk condogin badan kedepan
int angkatPahaAnjay = 15;//Variabel untuk angkat paha
int center = 150;
int center2 = 180;

boolean isActive = false;
boolean isread = true;
boolean isKuda = false;
boolean isKritis = false;
boolean step = false;
boolean isStarted = false;
boolean isIdle = true;
boolean audioState = false; 
boolean isdelay = true;
//=====================================================
static float alpha = 0.2;
double data_filtered[] = {0,0};
double data[] = {0, 0};
static int n = 1;
int sensorPin = 6;    // select the input pin for the potentiometer
//int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

//======================================================
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int total_sebelum = 0;
int total_sesudah = 0;
int total = 0;                  // the running total
int average = 0;                // the average
int average_sebelum = 0;
int average_sesudah = 0;

int inputPin = 1;              // analog input pin

//=======================================================

uint32 startTime;
uint32 now;
uint32 deltaTime;






//=====================================================
word syncPacket[36] = { // ID, POS, SPEED
  7, 2048, SPEED, // RIGHT
  9, 2048, SPEED,
  11, 2048, SPEED,
  13, 2048, SPEED,
  15, 2048, SPEED,
  17, 2048, SPEED,
  8, 2048, SPEED, // LEFT
  10, 2048, SPEED,
  12, 2048, SPEED,
  14, 2048, SPEED,
  16, 2048, SPEED,
  18, 2048, SPEED,
};

word syncPacket2[24] ={  
  1, 2048, SPEED, // tangan kanan
  3, 2048, SPEED,
  5, 2048, SPEED,
  2, 2048, SPEED, // tangan kiri
  4, 2048, SPEED,
  6, 2048, SPEED,
  19, 2048, SPEED, //leher
  20, 2048, SPEED, // kepala
};

word oldSyncPacket[36] = { // ID, POS, SPEED
  7, 2048, SPEED, // RIGHT
  9, 2048, SPEED,
  11, 2048, SPEED,
  13, 2048, SPEED,
  15, 2048, SPEED,
  17, 2048, SPEED,
  8, 2048, SPEED, // LEFT
  10, 2048, SPEED,
  12, 2048, SPEED,
  14, 2048, SPEED,
  16, 2048, SPEED,
  18, 2048, SPEED,
};

word oldSyncPacket2[24] = {
  1, 2048, SPEED, // tangan kanan
  3, 2048, SPEED,
  5, 2048, SPEED,
  2, 2048, SPEED, // tangan kiri
  4, 2048, SPEED,
  6, 2048, SPEED,
  19, 2048, SPEED, //leher
  20, 2048, SPEED, // kepala
};

//====================================================
void updateSyncPacket() {
  memcpy(oldSyncPacket, syncPacket, sizeof(syncPacket));
  for(int i = 0; i < 6; i++) {
    syncPacket[(i * 3) + 1] = convertAngle(angleRight[i], 0); 
    syncPacket[((i + 6) * 3) + 1] = convertAngle(angleLeft[i], 1);

  }
}
void updateSyncPacket2() {
  memcpy(oldSyncPacket2, syncPacket2, sizeof(syncPacket2));
  //syncPacket2[31] = convertAngle(body[0],0); //pinggang
  for(int j=0;j<2;j++){
    syncPacket2[((j+6)*3) + 1] = convertAngle2(body[j]);//leher&kepala 
  }
  for(int k=0;k<3;k++){
    syncPacket2[((k+0)*3)+1] = convertAngle2(handRight[k]);
    syncPacket2[((k+3)*3)+1] = convertAngle2(handLeft[k]);
  }
}


int convertAngle(float angle, boolean mirror) {
  int temp = map(angle, 0, 360, 0, 4095);
  if(mirror) {
    return (4095 - temp);
  }
  return temp;
}

int convertAngle2(float angle)
{
  angle = map(angle,0,360,0,4095);
  return angle;
}

void Leg(float x, float y, float z, float angle, float setA2, float a[6], boolean mirror)
{
  float r0, r1, r2, B, aX, aZ, g1, g2, g3, tempAngle;

  tempAngle = angle - 180;
  if(mirror) {
    z = -z;
    angle = -angle;
    tempAngle = -tempAngle;
  }
  if(mirror){ //mirror -> left
    r0 = sqrt(z * z + x * x);
    B = atan2(-z, -x) * PHI - tempAngle;
    aX = r0 * cos(B / PHI);
    aZ = r0 * sin(B / PHI);

    r1 = sqrt(y * y + aZ * aZ);
    r2 = sqrt(r1 * r1 + aX * aX);
    if(r2 > (left1 + left2)) r2 = left1 + left2;
    g1 = asin(aX / r2) * PHI;
    g3 = acos((left1 * left1 + left2 * left2 - r2 * r2) / (2 * left1 * left2)) * PHI;
    g2 = acos((left1 * left1 + r2 * r2 - left2 * left2) / (2 * left1 * r2)) * PHI;
  }
  else{    //no mirror -> right
    r0 = sqrt(z * z + x * x);
    B = atan2(-z, -x) * PHI - tempAngle;
    aX = r0 * cos(B / PHI);
    aZ = r0 * sin(B / PHI);

    r1 = sqrt(y * y + aZ * aZ);
    r2 = sqrt(r1 * r1 + aX * aX);
    if(r2 > (right1 + right2)) r2 = right1 + right2;
    g1 = asin(aX / r2) * PHI;
    g3 = acos((right1 * right1 + right2 * right2 - r2 * r2) / (2 * right1 * right2)) * PHI;
    g2 = acos((right1 * right1 + r2 * r2 - right2 * right2) / (2 * right1 * r2)) * PHI;
  }

  a[0] = angle;
  a[1] = atan2(aZ, y) * PHI;
  a[2] = -(g1 + g2);
  a[3] = 180 - g3;
  a[4] = a[2] + a[3];
  a[5] = a[1];
  a[2] = -(g1 + g2)-10;
  a[2] -= setA2;

  for(int i = 0; i < 6; i++) {
    a[i] += 180;
  }
}

//void Hand(float s1,float s3,float s25,float s27,float s29, boolean mirror)// mulai dari bahu ke ujung tangan, mirror menentukan : 0 = kanan, 1=kiri
void Hand(float s1,float s3,float s5, boolean mirror)// mulai dari bahu ke ujung tangan, mirror menentukan : 0 = kanan, 1=kiri
{
  float temp[3]= {
    s1,s3,s5};

  if(mirror){
    for(int i=0; i<3;i++)
    {
      temp[i]=360-temp[i];
    }
    memcpy(handLeft, temp, sizeof(temp));
  }
  else{
    memcpy(handRight, temp, sizeof(temp));
  }

}

void Body(float leher,  float kepala)
{
  float temp[2]={
    leher, kepala                      };
  memcpy(body,temp,sizeof(temp));
}



void moveOn(uint32 delayTime) {
  updateSyncPacket();
  word syncPacketTemp[36];
  memcpy(syncPacketTemp, oldSyncPacket, sizeof(oldSyncPacket));

  updateSyncPacket2();
  word syncPacketTemp2[24];
  memcpy(syncPacketTemp2, oldSyncPacket2, sizeof(oldSyncPacket2));

  delayTime *= 1000;

  if (isStarted == true){
    if (isKritis == false){
      //handler_led();
    }
  }
  //counter++; 
  //while(!audioState);

  startTime = micros();
  now = startTime;
  deltaTime = 0;
  SerialUSB.println("Mulai gerakan");

  while(1) {
    for(int j = 0; j < 12; j++) {
      syncPacketTemp[(j * 3) + 1] = map(deltaTime, 0, delayTime, oldSyncPacket[(j * 3) + 1], syncPacket[(j * 3) + 1]);
      if(j<8){
        syncPacketTemp2[(j * 3) + 1] = map(deltaTime, 0, delayTime, oldSyncPacket2[(j * 3) + 1], syncPacket2[(j * 3) + 1]);
      }
    }
    Dxl1.setPacketType(DXL_PACKET_TYPE1);
    Dxl1.syncWrite(30, 2, syncPacketTemp, 36);
//    Dxl.setPacketType(DXL_PACKET_TYPE2);      Protocol 2.0
//    Dxl.syncWrite(30, 2, syncPacketTemp2, 24);  
    Dxl.setPacketType(DXL_PACKET_TYPE1);
    Dxl.syncWrite(30, 2, syncPacketTemp2, 24);
    
    now = micros();
    deltaTime = now - startTime;
    
    getAudioState();
    
    if (isStarted == true)
    {
      if(isKritis == false)
      {
        if(audioState == false)
        {
          while(1)
          {
            getAudioState();
            if(audioState == true)
            {
              now = micros();
              startTime = now - deltaTime;
              break;
            }
          }
        }
      }
    }
//    if (isStarted == true){
//      if (isKritis == false){
//        handler_led();
//      }
//    }
    
    if(deltaTime > delayTime) {
      SerialUSB.println("Selesai gerakan");
      break;
    }
  }
}

void setup(){
  pinMode(BOARD_LED_PIN, OUTPUT);
  Serial2.begin(9600);
  Dxl.begin(3);
  Dxl1.begin(3);
  Dxl1.setPacketType(DXL_PACKET_TYPE1);
  Dxl.setPacketType(DXL_PACKET_TYPE1);
  Leg(0, normalFootHeight, -tiltOffset, 0, 0, angleLeft, 1);
  Leg(0, normalFootHeight, tiltOffset, 0, 0, angleRight, 0);
  Hand(center,center,center,1);
  Hand(center,center,center,0);
  Body(180,150);
  moveOn(1000);
  delay(1000);
  Leg(0, normalFootHeight, -tiltOffset, 0, 0, angleLeft, 1);
  Leg(0, normalFootHeight, tiltOffset, 0, 0, angleRight, 0);
  Hand(180,180,150,1);
  Hand(180,180,150,0);
  moveOn(delayTime); 
  Body(180,190);
  delay(1000);
  kuda(30);
}
void loop(){
  //========================================================
  //Masukan Gerakan disini 
//  isStarted = true;
//  isKritis = true;
//  do{
//    Hand(180,180,180,0); 
//    Hand(180,180,180,1); 
//    Body(180,180);
//    moveOn(1000);
//  Serial2.println("Siap");
//  delay(1000);
//  } while(0);
//  isKritis = false;
//isKritis = true;
//      //setComplianceKaki(10,18,20);
//      Leg(0,normalFootHeight, walkTilt,0, 0, angleLeft,1);
//      Leg(0,normalFootHeight, walkTilt,0, 0, angleRight,0);
//      moveOn(delayTime); 
//   
//      //setComplianceKaki(10,18,8);
//      Leg(walkDistanceOffset, normalFootHeight,walkTilt,0, 0, angleLeft,1); 
//      Leg(walkDistanceOffset, walkFootLift,walkTilt,0, 0, angleRight,0);
//      moveOn(delayTime);
//       
//      Leg(walkDistanceOffset + (-walkDistance),normalFootHeight,walkTilt,0, 0, angleLeft,1); 
//      Leg(walkDistanceOffset + walkDistance, walkFootLift,walkTilt,0, 0, angleRight,0);
//      moveOn(delayTime); 
//      
//      //setComplianceKaki(10,18,15);
//      Leg(walkDistanceOffset + (-walkDistance),normalFootHeight,0,0,0,angleLeft,1);
//      Leg(walkDistanceOffset + walkDistance,normalFootHeight,0,0, 0,angleRight,0);
//      moveOn(delayTime);
//      
//      //complianceKakiAwal();
//      Leg(walkDistanceOffset + (-walkDistance),normalFootHeight+20,0,0,0,angleLeft,1);
//      Leg(walkDistanceOffset + walkDistance,normalFootHeight+20,0,0,0,angleRight,0);
//      moveOn(delayTime);
//      
//      isKritis = false;
//   

//for(int i=0; i < 10; i++){
//  run();
//}
//  delay(200);
//for(int i=0; i < 3; i++){
//  left();
//
//}
//
//kuda(50);
//delay(500);
//for(int i=0; i < 5; i++){
//  run();
//}
//delay(500);
//  delay(300);
//for(int i=0; i < 3; i++){
//  right();
//  isKuda = false;
//  kuda(50);
//}
//  delay(1000);
//for(int i=0; i < 3; i++){
//  rig();
//  
//}  
//isKuda = false;
//  kuda(50);
//delay(1000);
//for(int i=0; i < 10; i++){
//  runs();
//}
//
//for(int i=0; i < 3; i++){
//  left();
//}
if (SerialUSB.available() and isread == true) {
  incomingByte = SerialUSB.read();
  isread = false;
}
if (arah != incomingByte){
  arah = incomingByte;
}
if (arah == 'G') {
  if (isdelay == true){delay(500); isdelay = false;}
  kuda(50);
  run();
  //run();
  isread = true;
     }
   // if it's an L (ASCII 76) turn off the LED:
else if (arah == 'L') {
  isKuda = false;
  kuda(30);
  left();
  isKuda = false;
  isdelay = true;
  isread = true;
     }
else if (arah == 'M') {
  isKuda = false;
  kuda(50);
  left();
  isKuda = false;
  isdelay = true;
  isread = true;
     }
else if (arah == 'R') {
  isKuda = false;
  kuda(30);
  lefts();
  isKuda = false;
  isdelay = true;
  isread = true;
     }     
else if (arah == 'O') {
  isKuda = false;
  kuda(30);
  lefts();
  isKuda = false;
  isdelay = true;
  isread = true;
     }    
else if (arah == 'S'){
  isread = true;
}
else {
  isKuda = false;
  toggleLED();
  delay(500);
   kuda(30);
   isKuda = false;
   isread = true;
}
    //kuda-kuda
//    if(isKuda == false){
//    isKritis = true;
//    kuda(30);
//    start();
//    }
    
//    for(int i = 0; i < 2; i++){
//    left();
//    }
//    for(int i = 0; i < 100; i++){
//    //lefts();
//    run();
//    }
//    kuda(30);
//    left();
//    delay(1000);
//    Leg(walkDistanceOffset + walkDistance ,normalFootHeight, walkTilt,0,0,angleLeft,1);
//    Leg(walkDistanceOffset + (walkDistance*3),normalFootHeight - 50, walkTilt,20,0,angleRight,0);
//    moveOn(delayTime); 
//    Leg(walkDistanceOffset + walkDistance ,normalFootHeight, walkTilt,0,0,angleRight,0);
//    Leg(walkDistanceOffset + (walkDistance*3),normalFootHeight - 50, walkTilt,20,0,angleLeft,1);
//    moveOn(delayTime); 

    
 //========================================================
}
void kuda(int a){
  if(isKuda == false){
    Leg(walkDistanceOffset + walkDistance - 10 ,normalFootHeight - a, 0,0,10,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance) - 10,normalFootHeight - a, 0,0,10,angleRight,0);
    Hand(150,150,150,0);
    Hand(150,150,150,1);
    moveOn(1000);
    isKuda = true;
    delay(500);
  }
}

void start(){
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 0 ,0,10,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*2)- 10,normalFootHeight - 60, 0, 0,10,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);
    moveOn(delayTime); 
//    delay(1000);
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 50, 0 ,0,10,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*3)- 10,normalFootHeight - 50, 0, 0,10,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);
    moveOn(delayTime); 
//    delay(1000);
}

void run(){
    //kiri 1
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 0 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 0, 0,15,angleLeft,1);
    Hand(150,150,150,1);
    Hand(210,150,150,0);
    moveOn(delayTime); 
 //   delay(1000);
    //kiri 2
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 5 ,15,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*2)- 10,normalFootHeight - 60, 0, 0,15,angleLeft,1);
    moveOn(delayTime/2); 
 //   delay(1000);
 
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 50, 5 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*3)- 10,normalFootHeight - 50, 0, 0,15,angleLeft,1);
    Hand(150,150,150,1);
    Hand(210,150,150,0);
    moveOn(delayTime); 
 
    //kanan 1
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 5 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 0, 0,15,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);
    moveOn(delayTime); 
//    delay(1000);
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 5 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*2)- 10,normalFootHeight - 60, 0, 0,15,angleRight,0);
    moveOn(delayTime/2); 
    //kanan 2
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 50, 0 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*3)- 10,normalFootHeight - 50, 0, 0,15,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);
    moveOn(delayTime); 
    
}

void test(){
     //kiri 1
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 0 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 0, 30,15,angleLeft,1);
    Hand(150,150,150,1);
    Hand(210,150,150,0);
    moveOn(delayTime); 
 //   delay(1000);
    //kiri 2
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 5 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*2)- 10,normalFootHeight - 60, 0, 0,15,angleLeft,1);
    moveOn(delayTime/2); 
 //   delay(1000);
 
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 50, 5 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*3)- 10,normalFootHeight - 50, 0, 0,15,angleLeft,1);
    Hand(150,150,150,1);
    Hand(210,150,150,0);
    moveOn(delayTime); 
 
    //kanan 1
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 5 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 0, 0,15,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);
    moveOn(delayTime); 
//    delay(1000);
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 5 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*2)- 10,normalFootHeight - 60, 0, 0,15,angleRight,0);
    moveOn(delayTime/2); 
    //kanan 2
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 50, 0 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*3)- 10,normalFootHeight - 50, 0, 0,15,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);
    moveOn(delayTime); 
   
}

void runs(){
    //kiri 1
 //   delay(1000);
    //kiri 2
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 5 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*2)- 10,normalFootHeight - 60, 0, 0,15,angleLeft,1);
    Hand(150,150,150,1);
    Hand(210,150,150,0);
    moveOn(delayTime/2); 
 //   delay(1000);
 
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 50, 5 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*3)- 10,normalFootHeight - 50, 0, 0,15,angleLeft,1);
    Hand(150,150,150,1);
    Hand(210,150,150,0);
    moveOn(delayTime); 
 
    //kanan 1
//    delay(1000);
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 5 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*2)- 10,normalFootHeight - 60, 0, 0,15,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);    
    moveOn(delayTime/2); 
    //kanan 2
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 50, 0 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*3)- 10,normalFootHeight - 50, 0, 0,15,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);
    moveOn(delayTime); 
    
}

void left(){
  isKuda = false;
    kuda(50);
    Leg(walkDistanceOffset + walkDistance - 10 ,normalFootHeight - 50, 30,0,10,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance) - 10,normalFootHeight - 55, 40,0,10,angleRight,0);
    moveOn(400);
    
    Leg(walkDistanceOffset + walkDistance - 10 ,normalFootHeight - 50, 20,0,10,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*4) - 10,normalFootHeight - 55, 40,0,10,angleRight,0);
    moveOn(400);
    
    Leg(walkDistanceOffset + walkDistance - 10 ,normalFootHeight - 50, 20,0,10,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*4) - 10,normalFootHeight -50, 30,0,10,angleRight,0);
    moveOn(400);
    delay(500);
}

void lefts(){
  isKuda = false;
    kuda(50);
    Leg(walkDistanceOffset + walkDistance - 10 ,normalFootHeight - 50, -30,0,10,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance) - 10,normalFootHeight - 55, -40,0,10,angleLeft,1);
    moveOn(400);
    
    Leg(walkDistanceOffset + walkDistance - 10 ,normalFootHeight - 50, -20,0,10,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*3) - 10,normalFootHeight - 55, -40,0,10,angleLeft,1);
    moveOn(400);
    
    Leg(walkDistanceOffset + walkDistance - 10 ,normalFootHeight - 50, -20,0,10,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*3) - 10,normalFootHeight -50, -30,0,10,angleLeft,1);
    moveOn(400);
  delay(500);
}

void right(){
  
    //kiri 1
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 0 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 0, 0,15,angleLeft,1);
    Hand(150,150,150,1);
    Hand(150,150,150,0);
    moveOn(delayTime); 
 //   delay(1000);
    //kiri 2
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 10 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, -30, 0,15,angleLeft,1);
    moveOn(delayTime/2); 
 //   delay(1000);
 
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 50, 10 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 50, -30, 0,15,angleLeft,1);
    moveOn(delayTime); 
}
void rights(){
  //kuda(50);
    //kanan 1
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 0 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 0, 0,15,angleRight,0);
    Hand(150,150,150,1);
    Hand(150,150,150,0);
    moveOn(delayTime); 
//    delay(1000);
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, -10 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 30, 0,15,angleRight,0);
    moveOn(delayTime/2); 
    //kanan 2
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 50, -10 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 50, 30, 0,15,angleRight,0);
    moveOn(delayTime); 
}

void rig(){ //geser ke kiri circle
  //kuda(50);
    //kanan 1
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 0 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 0, 0,15,angleRight,0);
    Hand(150,150,150,1);
    Hand(150,150,150,0);
    moveOn(delayTime); 
//    delay(1000);
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, -10 ,0,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 30, 0,15,angleRight,0);
    moveOn(delayTime/2); 
    //kanan 2
    Leg(walkDistanceOffset + walkDistance*3 - 10,normalFootHeight - 50, -10 ,20,15,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 50, 30, 0,15,angleRight,0);
    moveOn(delayTime); 
}

////////////////////////////////////////////////////////////////////////////////////////
////////// YANG INI MAD ///////////////////////////////////////////////////////////////
void handler_led(){
  
  while(1){
    getAudioState();
    if(audioState){
      break;
    }
  }
}

void getAudioState(){
   // read the value from the sensor:
   sensorValue = max(analogRead(sensorPin),0);
   // Low Pass Filter
   data_filtered[n] = alpha * sensorValue + (1 - alpha) * data_filtered[n-1];
//   if(data_filtered[n] > 30){
//     data_filtered[n] = 30 ;
//   }
    
   // Store the last filtered data in data_filtered[n-1]
   data_filtered[n-1] = data_filtered[n];
  
    
//   for(int index = 0; index < numReadings; index++){
//    // Read from the sensor:
//    readings[index] = max(analogRead(inputPin),0);
//    // Add the reading to the total:
//    total = total + readings[index];
//   }
//
//    // Calculate the average:
//    average = total / numReadings;
//    // Send it to the computer (as ASCII digits)
//    
//    //Reset total 
//    total = 0;
//    
//    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
//    readings[thisReading] = 0;
//    }



//  SerialUSB.print("Filtered data = ");
//  SerialUSB.println(data_filtered[n]);
//  delay(100);
  
  if( //average == 0
     (data_filtered[n] <= 5 && data_filtered[n-1] <= 5) || 
     (data_filtered[n] >= 50 && data_filtered[n-1] >= 50)
   ){
     audioState = false; 
     digitalWrite(BOARD_LED_PIN, HIGH);
//     SerialUSB.println("Mati Boy");
     //Serial3.write("0");
    }
  else{
    audioState = true;
    digitalWrite(BOARD_LED_PIN, LOW);
//    SerialUSB.print("Nyala Boy: ");
//    SerialUSB.println(data_filtered[n]);
    //Serial3.write("1");
  }
  
}


/////////////////////////////////////////////////////////////////////////////////////////




