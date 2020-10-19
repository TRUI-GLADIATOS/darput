


//=====================KRI Testing Data====================
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
//============================================================
//============================================================

//===========================================================
//====================KRI Testing Function=====================
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

//==================================================================
//================================================================


//////////////////////////////////////////////////////////////////
//////////////////////////Kinematics Learn Data///////////////////

int i = 0;
int j;
int num;

int cekData = 0;

//dibawah ini adalah blok data secara keseluruhan
//contoh data yg benar: x100;y200;z300;a050;A030
int DATA[30];
char x,y,z,a,A; //a itu 
int numx[3],numy[3],numz[3],numa[3],numA[3];//,numa[3],nummirror;
int Numx,Numy,Numz,Numa,NumA;//,Numa,Nummirror;
  // x = maju mundur
  // y = atas bawah
  // z = kanan kiri
  // a = rotasi kaki
  // A = kemiringan (tilt)
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////
////////////////////Kinematics Learn Function/////////////////
int connectSerial(int i, int value[]) {
  for(j = 0; j <= i; j++) {
    if(value[j] == 115 ) {
      if(value[j+1] == 101) {
        if(value[j+2] == 114) {
          if(value[j+3] == 105) {
            if(value[j+4] == 97) {
              if(value[j+5] == 108) {
              SerialUSB.println("Serial connection established");
              delay(3000);
              }
            }
          }
        }
      }
    }  
  }
  for(j = 0; j <= i; j++) {
     if(value[j] == 101 ) {
       if(value[j+1] == 120 ) {
         if(value[j+2] == 105 ) {
           if(value[j+3] == 116 ) {
             SerialUSB.println("Disconnecting Serial Connection");
             delay(3000);
             //dibawah ini untuk nge-reset ke nilai 0 untuk seluruh nilai data[]
             value[j] = 0;
             value[j+1] = 0;
             value[j+2] = 0;
             value[j+3] = 0; 
           }
         }
       }
     }
   }
}

void getData(int value[]) {
   while(SerialUSB.available()) { //ngambil data selama serialUSB ada data pending
    //print it out though USART2(RX2,TX2)

    value[i] = SerialUSB.read(); //ambil data one by one
    SerialUSB.print((char)value[i]); //print data dalam blok memori data[]
    //                ^ untuk nampilin ASCII dalam bentu char. kalau (int) bakal nampilin ASCII dalam bentuk desimal
    SerialUSB.print("   ");
    SerialUSB.println(i); //print nilai blok memori saat itu
    i++;  //nambah nilai blok memori untuk memasukkan data di blok data[] berikutnya
  }
  i = 0; //SANGAT PENTING!! untuk nge reset blok data mulai dari data[0] lagi
}

int k, period = 100;
unsigned long time_now = 0;
int kepala[3] = {0};
int kepala2,kepalaAkhir, ledValue;
int valueKepala(int value[]) {
  k = 0;
  while(SerialUSB.available()) { //ngambil data selama serialUSB ada data pending
    //print it out though USART2(RX2,TX2)

    value[k] = SerialUSB.read(); //ambil data one by one
    SerialUSB.print((char)value[i]); //print data dalam blok memori data[]
    //                ^ untuk nampilin ASCII dalam bentu char. kalau (int) bakal nampilin ASCII dalam bentuk desimal
    SerialUSB.print("   ");
    SerialUSB.println(k); //print nilai blok memori saat itu
    k++;  //nambah nilai blok memori untuk memasukkan data di blok data[] berikutnya
  }
  
  return (value[0]-48)*100 + (value[1]-48)*10 + (value[2]-48);
  k = 0; //SANGAT PENTING!! untuk nge reset blok data mulai dari data[0] lagi
  delay(1000);
}

int processData(int data[]) {
  if(DATA[0] == 0 && DATA[1] == 0 && DATA[2] == 0) return 0;//bila return 0, data[] masih kosong
  
  x = DATA[0];
  if(x != 'x')return -1;//cek kondisi, data pertama yg diterima harus x
  numx[0] = DATA[1] - 48;
  numx[1] = DATA[2] - 48;
  numx[2] = DATA[3] - 48;
  Numx = numx[0]*10*10 + numx[1]*10 + numx[2];
  
  if(DATA[4] != 59)return -1;//kalo datanya NULL, ga dianggap salah
  
  y = DATA[5];//y ngasilin char dari hasil int nya ASCII karna typenya y itu char
  if(y != 'y')return -1;//cek kondisi, data kelima harus y atau '121' which is ASCII nya y;
  numy[0] = DATA[6] - 48;
  numy[1] = DATA[7] - 48;
  numy[2] = DATA[8] - 48;
  Numy = numy[0]*10*10 + numy[1]*10 + numy[2];
  
  if(DATA[9] != 59) return -1;//kalo datanya NULL, ga dianggap salah
  
  z = DATA[10];
  if(z != 'z')return -1;//cek data, data kesepuluh harus z
  numz[0] = DATA[11] - 48;
  numz[1] = DATA[12] - 48;
  numz[2] = DATA[13] - 48;
  Numz = numz[0]*10*10 + numz[1]*10 + numz[2];
  
  if(DATA[14] != 59) return -1;//kalo datanya NULL, ga dianggap salah
  
  a = DATA[15];
  if(a != 'a')return -1;//cek data, data kesepuluh harus z
  numa[0] = DATA[16] - 48;
  numa[1] = DATA[17] - 48;
  numa[2] = DATA[18] - 48;
  Numa = numa[0]*10*10 + numa[1]*10 + numa[2];
  
  if(DATA[19] != 59) return -1;//kalo datanya NULL, ga dianggap salah
  
  A = DATA[20];
  if(A != 'A')return -1;//cek data, data kesepuluh harus z
  numA[0] = DATA[21] - 48;
  numA[1] = DATA[22] - 48;
  numA[2] = DATA[23] - 48;
  NumA = numA[0]*10*10 + numA[1]*10 + numA[2];
  
  if(DATA[24] != 59) return -1;//kalo datanya NULL, ga dianggap salah
  
  return 1;//bila return 1; data yang diterima formatnya benar
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////


void setup(){
  SerialUSB.begin(); //inisiasi koneksi serialUSB
  
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
  Body(180,120);//180, 190
  delay(1000);
  kuda(30);
  
  //buat bediri tegak
//   Leg(0, normalFootHeight, -tiltOffset, 0, 0, angleLeft, 1);
//  Leg(0, normalFootHeight, tiltOffset, 0, 0, angleRight, 0);
//  Hand(center,center,center,1);
//  Hand(center,center,center,0);
//  Body(180,150);
//  moveOn(delayTime);


}


void loop(){
 
//////////////////////////////////////////////////////////////////////
////////////////Vision learn loop/////////////////////////////



/*
  int time_now = millis();
  int period = 500;
  int kepalaAkhir;
  int kepala2;
  kepala2 = valueKepala(kepala);
  if(kepala2 == -5328) kepala2 += 5328;
  SerialUSB.print("kepala: ");
  SerialUSB.println(kepala2);
*/
 
if(millis() >= time_now + period) {
  time_now += period;
  kepala2 = valueKepala(kepala);
  if(kepala2 == -5328) kepala2 += 5328;
  if(kepala2 < 0 || kepala2 > 640) kepala2 = kepalaAkhir;
  kepalaAkhir = map(kepala2,640,0,100,250);
  ledValue = map(kepala2,0,640,0,255);
  SerialUSB.print("kepala: ");
  SerialUSB.println(kepalaAkhir);
  Body(kepalaAkhir,130);
  moveOn(delayTime);
}
/////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
 
  
////////////////////////////////////////////////////////////////
//////////////////Kinematics Learn loop/////////////////////////
  // when you typed any character in terminal
  //if(SerialUSB.available()) {
    
  //selama gak ada data yg masuk, data gak berubah
  
  /*
  getData(DATA);
  connectSerial(i,DATA); /*jika data[0] sampe data[5] 
  isinya s,e,r,i,a,l maka akan kekoneksi ke serial. jika data[0]
  sampe data[3] isinya e,x,i,t makan akan diskonek*/
 
 /* 
  cekData = processData(DATA);
  SerialUSB.print("Status kebenaran data: ");
  SerialUSB.println(cekData);
 */
 /* 
  //Kode dibawah ini akan nge-reset seluruh data apabila mendapatkan data yang tidak dikenal
  if(cekData == -1) {
    SerialUSB.println("Unrecognized type of data");
    for(j = 0; j<=20; j++) {
     data[j] = 0; //hapus semua data yg diterima sebelumnya
    }
    for(j = 0; j<=2; j++) {
      numx[j] = 0;  //hapus semua nilai num
      numy[j] = 0;
      numz[j] = 0;
      numa[j] = 0;
      numA[j] = 0;
    }  
    Numx = 0;
    Numy = 0;  //hapus semua nilai Num
    Numz = 0;
    Numa = 0;
    NumA = 0;
    x = 0;  //hapus semua character
    y = 0;
    z = 0;
    a = 0;
    A = 0;
 */  

/* 
  //x000;y190;z015;a000;A000;
  //buat bediri tegak
  //          v 190              v 15
    Leg(0, normalFootHeight, -tiltOffset, 0, 0, angleLeft, 1);
    Leg(0, normalFootHeight, tiltOffset, 0, 0, angleRight, 0);
  //      v 150  
    Hand(center,center,center,1);
    Hand(center,center,center,0);
    Body(180,150);
    moveOn(delayTime);
  }
 */

/* 
  if(cekData == 0) {
    SerialUSB.println("Pending for data");
    
      //buat bediri tegak
    Leg(0, normalFootHeight, -tiltOffset, 0, 0, angleLeft, 1);
    Leg(0, normalFootHeight, tiltOffset, 0, 0, angleRight, 0);
    Hand(center,center,center,1);
    Hand(center,center,center,0);
    Body(180,150);
    moveOn(delayTime);
  } 
  if(cekData == 1) {
      //buat bediri tegak
    //Leg(Numx, Numy, -Numz, Numa, NumA, angleLeft, 1);
    //Leg(Numx, Numy, Numz, Numa, NumA, angleRight, 0);
    Hand(center,center,center,1);
    Hand(center,center,center,0);
    Body(Numx,Numy);
    moveOn(delayTime);  
  }
  
  SerialUSB.print(x);
  SerialUSB.println(Numx);
  SerialUSB.print(y);
  SerialUSB.println(Numy);
  SerialUSB.print(z);
  SerialUSB.println(Numz);
  SerialUSB.print(a);
  SerialUSB.println(Numa);
  SerialUSB.print(A);
  SerialUSB.println(NumA);
*/ 
  //delay(1000);
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$Kodingan krisna$$$$$$$$$$$$$$$$$

/*
if(millis() >= time_now + period) { //buat kepala ikutin arah vision
  time_now += period;
  kepala2 = valueKepala(kepala);
  if(kepala2 == -5328) kepala2 += 5328;
  if(kepala2 < 0 || kepala2 > 640) kepala2 = 320;
  kepalaAkhir = map(kepala2,0,640,100,250);
  ledValue = map(kepala2,0,640,0,255);
  SerialUSB.print("kepala: ");
  SerialUSB.println(kepalaAkhir);
  Body(kepalaAkhir,130);
  moveOn(delayTime);
}
*/


void runsTest() { //buat lari kedepan
      //kiri 1
 //   delay(1000);
    //kiri 2
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 35, 5 ,0,26,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*2)- 10,normalFootHeight - 65, 0, 0,26,angleLeft,1);
    Hand(150,150,150,1);
    Hand(210,150,150,0);
    moveOn(delayTime); 
 //   delay(1000);
 
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 55, 5 ,0,26,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance*3)- 10,normalFootHeight - 55, 0, 0,26,angleLeft,1);
    Hand(150,150,150,1);
    Hand(210,150,150,0);
    moveOn(delayTime); 
 
    //kanan 1
//    delay(1000);
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 35, 5 ,0,26,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*2)- 10,normalFootHeight - 65, 0, 0,26,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);    
    moveOn(delayTime); 
    //kanan 2                                                         
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 55, 0 ,0,26,angleLeft,1);
    Leg(walkDistanceOffset + (walkDistance*3)- 10,normalFootHeight - 55, 0, 0,26,angleRight,0);
    Hand(150,150,150,0);
    Hand(210,150,150,1);
    moveOn(delayTime);
}


void rigs(){ //geser ke kiri circle
  //kuda(50);
    //kanan 1
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 0 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, 0, 0,15,angleLeft,1);
    Hand(150,150,150,1);
    Hand(150,150,150,0);
    moveOn(delayTime); 
//    delay(1000);
    Leg(walkDistanceOffset + walkDistance - 10,normalFootHeight - 40, 10 ,0,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 60, -30, 0,15,angleLeft,1);
    moveOn(delayTime/2); 
    //kanan 2
    Leg(walkDistanceOffset + walkDistance*3 - 10,normalFootHeight - 50, 10 ,-20,15,angleRight,0);
    Leg(walkDistanceOffset + (walkDistance)- 10,normalFootHeight - 50, -30, 0,15,angleLeft,1);
    moveOn(delayTime); 
}



//
//int period1 = 35000;
//   int period2 = 5000;
//   int period3 = 20000;
//   int period4 = 3000;
//   int period5 = 25000;
//   int period6 = 0;
//   int period7 = 0;
//   int period8 = 50000;
//   int sekarang,tutup;
//  
//  
//      isdelay = true;
//      if (isdelay == true){delay(1000); isdelay = false;}
//      kuda(50);
//      tutup = millis() + period1;
//      while(/*arah == 'W'*/true) {//lari kedepan
//        //kuda(50);
//        runsTest();
//        isread = true;
//        SerialUSB.print("maju");
//        if(millis() > tutup) break;
//      }
//      isdelay = true;
//      if (isdelay == true){delay(1000); isdelay = false;}
//      kuda(50);
//      tutup = millis() + period2;
//      while(/*arah == 'F'*/true) {//kepiting kiri
//        //kuda(50);
//        rig();
//        isread = true;
//        SerialUSB.print("kiri");
//        if(millis() > tutup) break;
//      }
//      isdelay = true;
//      if (isdelay == true){delay(1000); isdelay = false;}
//      kuda(50);
//      tutup = millis() + period3;
//      while(/*arah == 'W'*/true) {//lari kedepan
//        //kuda(50);
//        runsTest();
//        isread = true;
//        SerialUSB.print("lurus");
//        if(millis() > tutup) break;
//      }
//      isdelay = true;
//      if (isdelay == true){delay(1000); isdelay = false;}
//      kuda(50);
//       tutup = millis() + period4;
//      while(/*arah == 'G'*/true) {//kepiting kanan
//        //kuda(50);
//        rig();
//        isread = true;
//        SerialUSB.print("kanan");
//        if(millis() > tutup) break;
//      }
//      isdelay = true;
//      if (isdelay == true){delay(1000); isdelay = false;}
//      kuda(50);
//      tutup = millis() + period5;
//      while(/*arah == 'R'*/true) {//muter ditempat
//        //kuda(50);
//        runsTest();
//        isread = true;
//        SerialUSB.print("puter");
//        if(millis() > tutup)break;
//      }
//      isdelay = true;
//      if (isdelay == true){delay(1000); isdelay = false;}
//      kuda(50);
//      tutup = millis() + period6;
//      while(/*arah == 'P'*/true) {
//        rig();
//        isread = true;
//        SerialUSB.print("lari");
//        if(millis() > tutup)break;
//      }
//      isdelay = true;
//      if (isdelay == true){delay(1000); isdelay = false;}
//      kuda(50);
//      tutup = millis() + period7;
//      while(/*arah == 'P'*/true) {
//        runsTest();
//        isread = true;
//        SerialUSB.print("lari");
//        if(millis() > tutup)break;
//      }
//      isdelay = true;
//      if (isdelay == true){delay(1000); isdelay = false;}
//      kuda(50);
//      tutup = millis() + period8;
//      while(/*arah == 'P'*/true) {
//        kuda(50);
//        isread = true;
//        SerialUSB.print("lari");
//        if(millis() > tutup)break;
//      }






//*****************************************************
//******************kodingan moes**********************

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


//***********************************************************
//***********************************************************

