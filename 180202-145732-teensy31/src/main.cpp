#include <Arduino.h>
#include <stdint.h>

#define UART_TXRTSE (2)
#define UART_TXRTSPOL (4)
#define SERIAL_PORT Serial2         //PORT TO SERVO
#define SERIAL_KOMPAS Serial3       

#define offsetAscii '0'
#define ledPin 13

void setup() 
{
  digitalWrite(ledPin, LOW);
  Serial.begin(1000000);
  SERIAL_PORT.begin(1000000);
  SERIAL_PORT.transmitterEnable(6);
  SERIAL_KOMPAS.begin(9600);
  pinMode(ledPin, OUTPUT);
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
  digitalWrite(ledPin, HIGH);
    penghitung = 0;                 //counter as a flag
    delay(5);                       //delay to wait for serial to completely receive data
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
    //SERIAL_PORT.flush();          //make sure all buffer in servo line is sent
    delay(5);                     //make sure all data from servo is arrived
    while (SERIAL_PORT.available()){      //check if any left data in servo line serial
      c = SERIAL_PORT.read();     //forward byte from servo
      Serial.write(c);            //to miniPC
    }
    Serial.flush();               //Make sure data is completely sent to USB
    digitalWrite(ledPin, LOW);
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
  }/*else if(){

  }*/

}

