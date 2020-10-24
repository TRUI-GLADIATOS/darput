
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


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
//$$$$$$$$$$$$$$$$$$$$get serial data$$$$$$$$$$$$$$$//

int i = 0;
int j;
int num;
int cekData = 0;

char d; //ini buat parameter kasi tau gerakannya!
int DATA[10];
int numh[3],numv[3];
int Numh,Numv;



void getData(int value[]) {
  i = 0;
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



int processData(int data[]) {
  if(DATA[0] == 0) return 0;//bila return 0, data[] masih kosong
  
  //if(DATA[0] != 'W' || DATA[0] != 'A' || DATA[0] != 'S' || DATA[0] != 'D') return -1;
 /* if(DATA[0] != 65) return -1;
  if(DATA[0] != 83) return -1;
  if(DATA[0] != 68) return -1;
  */
  d = DATA[0]; //terima data arah gerakan
  if(d != 'W') { //cek kebenaran data arah gerakan 
    if(d != 'A') {
      if(d != 'S') {
        if(d != 'D') return -1;
      }
    }
  }
  
  if(DATA[1] != ';')return -1;//cek kondisi, data pembagi yg diterima harus ;
  
  numh[0] = DATA[2] - 48;
  numh[1] = DATA[3] - 48;
  numh[2] = DATA[4] - 48;
  Numh = numh[0]*10*10 + numh[1]*10 + numh[2];
  
  if(DATA[5] != ';')return -1;//kalo datanya NULL, ga dianggap salah
  
  //y = DATA[5];//y ngasilin char dari hasil int nya ASCII karna typenya y itu char
  //if(y != 'y')return -1;//cek kondisi, data kelima harus y atau '121' which is ASCII nya y;
  numv[0] = DATA[6] - 48;
  numv[1] = DATA[7] - 48;
  numv[2] = DATA[8] - 48;
  Numv = numv[0]*10*10 + numv[1]*10 + numv[2];
  
  if(DATA[9] != ';') return -1;//kalo datanya NULL, ga dianggap salah
    
  return 1;//bila return 1; data yang diterima formatnya benar
}


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$//




//*************fungsi kepala********************//
//variable buat koneksi serial
int k, period = 500; //period buat dia nerima data buat kepala selama berapa milisec sekali
unsigned long time_now = 0; //buat ambil data time saat itu
int kepala[3] = {0}; //data buat kepala dari array ASCII
int kepala2,kepalaAkhir, ledValue;

//buat filtering
//int alpa = 0.8;

int valueKepala(int value[]) {
  k = 0;
  /*
  while(SerialUSB.available()) { //ngambil data selama serialUSB ada data pending
    //print it out though USART2(RX2,TX2)

    value[k] = SerialUSB.read(); //ambil data one by one
    SerialUSB.print((char)value[k]); //print data dalam blok memori data[]
    //                ^ untuk nampilin ASCII dalam bentu char. kalau (int) bakal nampilin ASCII dalam bentuk desimal
    SerialUSB.print("   ");
    SerialUSB.println(k); //print nilai blok memori saat itu
    k++;  //nambah nilai blok memori untuk memasukkan data di blok data[] berikutnya
  }
  */
  return (value[0]-48)*100 + (value[1]-48)*10 + (value[2]-48);
  k = 0; //SANGAT PENTING!! untuk nge reset blok data mulai dari data[0] lagi
  delay(1000);
}




int kepalaCari(char input, int sekarang, int period2) {
  int p = 0; 
  kepala2 = 320;
  while(input == 'S') {
    if(millis() >= sekarang + period2) {
      //kepala2 = 320;
      sekarang += period; //ini parameter buat nerima data selama berapa detik sekali
      
      if(kepala2 == -5328) kepala2 += 5328;
      //if(kepala2 < 0 || kepala2 > 640) kepala2 = kepalaAkhir;
      //kepala2 = 320;
      kepalaAkhir = map(kepala2,640,0,140,220);
      
      SerialUSB.print("kepala akhir: ");
      SerialUSB.println(kepalaAkhir);
      //buat tengok kepalanya
      if(p > 5) kepala2 += 70;
      else if(p <= 5) kepala2 -= 70;
      
      
          //rumus filternya (gatau kenapa gabisa)
      //kepalaAkhir = (alpa*kepalaAkhir) + ((1-alpa)*kepala2);
      kepalaAkhir = map(kepala2,640,0,140,220);  
      SerialUSB.print("kepala akhir: ");
      SerialUSB.println(kepalaAkhir);
      SerialUSB.println(p);
      Body(kepalaAkhir,130);
      moveOn(delayTime);
      p++;
      
      //dibawah ini buat keluar dari loopnya kalo datanya bukan S
      getData(DATA);
      cekData = processData(DATA); 
      input = DATA[0];
      
      if(p == 17) {
        p = 0; //biar tengoknya bisa berulang kali
        kepala2 = 320;
      }
    }
   
  }
  return kepalaAkhir;
}
//**************************************************//


void setup() {
  // put your setup code here, to run once:
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

int time = millis(); //ini dah bener disinii!!
void loop() {
  //time_now = millis(); -> jangan taroh disini
  getData(DATA);
  cekData = processData(DATA);
  SerialUSB.print("Status kebenaran data: ");
  SerialUSB.println(cekData);
  
   //Kode dibawah ini akan nge-reset seluruh data apabila mendapatkan data yang tidak dikenal
  if(cekData == -1) {
    SerialUSB.println("Unrecognized type of data");
    for(j = 0; j<=10; j++) {
     DATA[j] = 0; //hapus semua data yg diterima sebelumnya
    }
    for(j = 0; j<=2; j++) {
      numh[j] = 0;  //hapus semua nilai num
      numv[j] = 0;
    }  
    Numh = 0;
    Numv = 0;  //hapus semua nilai Num
    d = 0;  //hapus semua character
  } 
  
  
  if(cekData == 0) SerialUSB.println("Pending for data");

  if(cekData == 1) {
 
    if(millis() >= time/*_now*/ + period) {
      //kepala2 = 320;
      time/*_now*/ += period; //ini parameter buat nerima data selama berapa detik sekali
      //kepala2 = valueKepala(kepala); 
      kepala2 = Numh;
      if(kepala2 == -5328) kepala2 += 5328;
      if(kepala2 < 0 || kepala2 > 640) kepala2 = kepalaAkhir;
      //kepala2 = 320;
      kepalaAkhir = map(kepala2,640,0,140,220);
      
      SerialUSB.print("kepala: ");
      SerialUSB.println(kepalaAkhir);
      Body(kepalaAkhir,130);
      moveOn(delayTime);
    }

    time_now = millis();
    if(d == 'S') {
     //taroh kodingan kepalaCari()
      kepalaCari(d, time_now, 500); 
    }
    
  }
  
  //SerialUSB.println(d);
  //SerialUSB.println(Numh);
  //SerialUSB.println(Numv);
  delay(100);
    
  
}




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


///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
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



