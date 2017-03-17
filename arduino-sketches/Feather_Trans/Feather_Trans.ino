/* RFM69 library and code by Felix Rusu - felix@lowpowerlab.com
// Get libraries at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!
// **********************************************************************************
// Copyright Felix Rusu, LowPowerLab.com
// Library and code by Felix Rusu - felix@lowpowerlab.com
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses></http:>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************/

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     100  // The same on all nodes that talk to each other
#define NODEID        2    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets

//Match frequency to the hardware version of the radio on your Feather
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   115200

/* for Feather 32u4 Radio
#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    4  // Pin 7 is IRQ 4!
#define RFM69_RST     4
*/

/* for Feather M0 Radio */
#define RFM69_CS      8
#define RFM69_IRQ     3
#define RFM69_IRQN    3  // Pin 3 is IRQ 3!
#define RFM69_RST     4

/* ESP8266 feather w/wing
#define RFM69_CS      2
#define RFM69_IRQ     15
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
#define RFM69_RST     16
*/

/* Feather 32u4 w/wing
#define RFM69_RST     11   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!)
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

/* Feather m0 w/wing 
#define RFM69_RST     11   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     6    // "D"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

/* Teensy 3.x w/wing 
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

#define LED           13  // onboard blinky
//#define LED           0 //use 0 on ESP8266
#define BUFFER_SIZE 61
#define TIMEOUT_SET 20

int16_t packetnum = 0;  // packet counter, we increment per xmission
char buffer[BUFFER_SIZE];
int buffer_idx = 0;
int timeout = TIMEOUT_SET;

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

void setup() {
  while (!Serial); // wait until serial console is open, remove if not tethered to computer. Delete this line on ESP8266
  Serial.begin(SERIAL_BAUD);

  Serial.println("Feather RFM69HCW Transmitter");
  
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  radio.encrypt(ENCRYPTKEY);
  
  pinMode(LED, OUTPUT);
  Serial.print("\nTransmitting at ");
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");
}


void loop() {
  delay(100);  // Wait 0.1 second between transmits, could also 'sleep' here!

//  char incomingByte = 0;
  int read_amt = 0;
  if (Serial.available() > 0) {
//    int buffer_len = 0;
      // read the incoming byte:
//      incomingByte = Serial.read();
    read_amt = Serial.readBytes(buffer, BUFFER_SIZE);

//      if (buffer_len = addToBuffer(incomingByte)) {
    radio.send(RECEIVER, buffer, read_amt);
//        radio.send(RECEIVER, buffer, buffer_len);
//        Serial.print("Sent: (full)");
//        for (int i = 0; i < buffer_len; i++)
//          Serial.print(buffer[i]);
//        Serial.print("\n");
//        return;
//        Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
//      }
//      if (radio.sendWithRetry(RECEIVER, &incomingByte, 1, 10)) { //target node Id, message as string or byte array, message length
//        Serial.println("OK");
//        Blink(LED, 50, 1); //blink LED 3 times, 50ms between blinks
//      }
//      Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
      // say what you got:
      
//      Serial.println(incomingByte);
  }
//  else if (buffer_idx != 0 && timeout <= 0) { //if we have remaining data, flush it out via Serial
//    Serial.println("timed out");
//    int buffer_len = addToBuffer(0);
//    radio.send(RECEIVER, buffer, buffer_len);
//    Serial.print("Sent: (");
//    Serial.print(buffer_len);
//    Serial.print(") [");
//    for (int i = 0; i < buffer_len; i++)
//      Serial.print(buffer[i]);
//    Serial.print("]\n");
//    Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
//    timeout = TIMEOUT_SET;
//  }
//  else if (buffer_idx != 0 && timeout > 0) {
//    Serial.println("timed count down");
//    timeout--;
//  }
    
//  char radiopacket[20] = "Hello World #";
//  itoa(packetnum++, radiopacket+13, 10);
//  Serial.print("Sending "); Serial.println(radiopacket);
  
//  if (radio.sendWithRetry(RECEIVER, radiopacket, strlen(radiopacket))) { //target node Id, message as string or byte array, message length
//    Serial.println("OK");
//    Blink(LED, 50, 1); //blink LED 3 times, 50ms between blinks
//  }

//  radio.receiveDone(); //put radio in RX mode
  Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
}

int addToBuffer(char c) {
  int ret = 0;
  if (c == 0) {
    ret = buffer_idx;
    buffer_idx = 0;
    return ret;
  }
  buffer[buffer_idx] = c;
  buffer_idx++;
//  if (c == '\n' || c == '\r') {
//    for (int i = buffer_idx; i < BUFFER_SIZE; i++)
//      buffer[i] = 0;
//    ret = buffer_idx;
//    buffer_idx = 0;
//  }
  if (buffer_idx >= BUFFER_SIZE) {
    buffer_idx = 0;
    ret = BUFFER_SIZE;
  }
  return ret;
}

void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i=0; i<loops; i++)
  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
