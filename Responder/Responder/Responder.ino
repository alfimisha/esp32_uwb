/*
   Copyright (c) 2024 by Ashutosh Dhekne <dhekne@gatech.edu>
   This program runs on the UWB device connected to the login device.  

   You are free to:
Share — copy and redistribute the material in any medium or format
The licensor cannot revoke these freedoms as long as you follow the license terms.
Under the following terms:
Attribution — You must give appropriate credit , provide a link to the license, and indicate if changes were made . 
You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
NonCommercial — You may not use the material for commercial purposes .
NoDerivatives — If you remix, transform, or build upon the material, you may not distribute the modified material.
No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
Notices:
You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation .

No warranties are given. The license may not give you all of the permissions necessary for your intended use. 
For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.
*/
#include "genericFunctions.h"
#include "RangingContainer.h"
#include <time.h>
#include<TimeLib.h>
#include<Wire.h>
//#include <Adafruit_NeoPixel.h>

//Debug levels
int DebugUWB_L1 = 0;
int DebugUWB_L2 = 0;
int DebugWebserverComms = 1;
int DebugCrypto = 1;
int DebugUWB_LRXTO = 0;

//NeoPixel
// Which pin on the Arduino is connected to the NeoPixels?
//#define NEOPIXELPIN        8 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
//#define NUMPIXELS 1 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
//Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);

//UWB Globals
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 19; // spi select pin

// packet sent status and count
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
volatile boolean sendComplete = false;
volatile boolean RxTimeout = false;
String message;

//UWB Messages
byte tx_poll_msg[MAX_POLL_LEN] = {POLL_MSG_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte rx_resp_msg[MAX_RESP_LEN] = {RESP_MSG_TYPE, 0x02, 0, 0, 0, 0};
byte tx_final_msg[MAX_FINAL_LEN] = {FINAL_MSG_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte rx_dist_msg[MAX_DIST_LEN] = {DIST_EST_MSG_TYPE, 0, 0, 0, 0, 0, 0};
byte tx_key_ack[MAX_POLL_LEN] = {SEND_KEYS_ACK_TYPE, 0, 0, 0, 0, 0, 0};
int response_counter = 0;
char rx_msg_char[200];
byte rx_packet[128];
char myDevID = 1;

//UWB Ranging
unsigned int seq;
uint64_t RespTs;
Ranging thisRange;
uint32_t dist;
int32_t dist_ss;
int32_t dist_ss2;
uint32_t dist_avg_formula;
char login_device;

//UWB State Machine
typedef enum states {STATE_IDLE, STATE_POLL, STATE_RESP_EXPECTED, STATE_FINAL_SEND, STATE_TWR_DONE, STATE_RESP_SEND, STATE_FINAL_EXPECTED, STATE_OTHER_POLL_EXPECTED, STATE_RESP_PENDING, STATE_DIST_EST_EXPECTED, STATE_DIST_EST_SEND, STATE_TIGHT_LOOP,
                     STATE_RECEIVE, STATE_PRESYNC, STATE_SYNC, STATE_ANCHOR, STATE_TAG, STATE_FIRST_START, STATE_OBLIVION, STATE_ACK_EXPECTED, STATE_SEND_KEY_IV, STATE_SEND_KEY_IV_ACK
                    } STATES;
volatile uint8_t current_state = STATE_IDLE;

//UWB Error Handling
int RX_TO_COUNT = 0;




//UWB Support Functions
void receiver(uint16_t rxtoval = 0 ) {
  received = false;
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(false);
  if (rxtoval > 0) {
    DW1000.setRxTimeout(rxtoval);
  } else {
    DW1000.setRxTimeout(rxtoval);
  }
  DW1000.startReceive();
}


void setup() {
  Serial.begin(9600);
  while (!Serial)
  {
    delay(10);
  }
  //Serial.print("Waiting for instructions...");
  //pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  //pixels.clear(); // Set all pixel colors to 'off'
  //pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  //pixels.setBrightness(50);
  //pixels.show();
  delay(500);

  //Initialize UWB
  randomSeed(analogRead(0));
  Serial.println(F("-----------------------------"));
  Serial.println(F("-->     UWB Responder     <--"));
  Serial.println(F("-----------------------------"));
  Serial.println("Free memory: ");
  Serial.println(freeMemory());
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) received messages
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveTimeoutHandler(handleRxTO);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);
  DW1000.attachSentHandler(handleSent);

  current_state = STATE_RECEIVE;
  receiver(TYPICAL_RX_TIMEOUT);
  
}


void handleSent() {
  // status change on sent success
  sendComplete = true;
  //Serial.println("Send complete");
}


void handleReceived() {
  RX_TO_COUNT = 0;
  // status change on reception success

  DW1000.getData(rx_packet, DW1000.getDataLength());
  //  Serial.println("Received something...");
  received = true;

//  byte2char(rx_packet, 24);
//  Serial.println(rx_msg_char);


}

void handleError() {
  if (current_state == STATE_RECEIVE)
  {
    current_state = STATE_RECEIVE;
  } else if(current_state == STATE_ANCHOR)
  {
    current_state = STATE_ANCHOR;
  }
  else {
    current_state = STATE_IDLE;
  }
  RxTimeout = true;
  error = true;
  Serial.println("ERROR");

}

void handleRxTO() {
  RX_TO_COUNT++;
  //Serial.println("RXTO");
  RxTimeout = true;
  if (DebugUWB_LRXTO == 1) {
    printState();
  }
}



void loop() {
  
  
  switch(current_state) {
    case STATE_IDLE:
      break;
    case STATE_RECEIVE:
    {
      if (received == true)
      {
        received = false;
        if (DebugUWB_L1 == 1) {
          Serial.println("State: STATE_RECEIVE");
        }
        /*pixels.clear(); // Set all pixel colors to 'off'
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        pixels.setBrightness(50);
        pixels.show();*/
  
        //Check the message type and take action accordingly.
        if (rx_packet[DST_IDX] == BROADCAST_ID && rx_packet[0] == POLL_MSG_TYPE) {
          seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
          //Send response after saving current information.
          uint64_t PollTxTime_64 = 0L;
          any_msg_get_ts(&rx_packet[POLL_MSG_POLL_TX_TS_IDX], &PollTxTime_64);
          thisRange.PollTxTime = DW1000Time((int64_t)PollTxTime_64);
          DW1000Time rxTS;
          DW1000.getReceiveTimestamp(rxTS);
          thisRange.PollRxTime = rxTS;
          seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
          //Change state so that a response can be sent soon.
          current_state = STATE_RESP_SEND;     
        }
      }
      break;
    }
    case STATE_RESP_SEND:
    {
      if (DebugUWB_L1 == 1) {
        Serial.println("State: STATE_RESP_SEND");
      }
      rx_resp_msg[DST_IDX] = rx_packet[SRC_IDX];
      rx_resp_msg[SRC_IDX] = myDevID;
      rx_resp_msg[SEQ_IDX] = seq & 0xFF;
      rx_resp_msg[SEQ_IDX + 1] = seq >> 8;
      //delay(35);
      generic_send(rx_resp_msg, sizeof(rx_resp_msg), POLL_MSG_POLL_TX_TS_IDX, SEND_DELAY_FIXED);
  
      while (!sendComplete);
      sendComplete = false;
      DW1000Time txTS;
      DW1000.getTransmitTimestamp(txTS);
      thisRange.RespTxTime = txTS;
      current_state = STATE_FINAL_EXPECTED;
      receiver(TYPICAL_RX_TIMEOUT);
      break;
    }
    case STATE_FINAL_EXPECTED:
    {
      if (received == true)
      {
        received = false;
        if (DebugUWB_L1 == 1) {
          Serial.println("State: STATE_FINAL_EXPECTED");
        }
        if (rx_packet[0] == FINAL_MSG_TYPE) {
          unsigned int recvd_resp_seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
          if (recvd_resp_seq == seq) {
            //pixels.clear(); // Set all pixel colors to 'off'
            //pixels.setPixelColor(0, pixels.Color(0, 0, 255));
            //pixels.setBrightness(50);
            //pixels.show();
            DW1000Time rxTS;
            DW1000.getReceiveTimestamp(rxTS);
            thisRange.FinalRxTime = rxTS;
                  
            uint64_t RespRxTime_64 = 0L;
            any_msg_get_ts(&rx_packet[FINAL_MSG_RESP_RX_TS_IDX], &RespRxTime_64);
            thisRange.RespRxTime = DW1000Time((int64_t)RespRxTime_64);
            uint64_t FinalTxTime_64 = 0L;
            any_msg_get_ts(&rx_packet[FINAL_MSG_FINAL_TX_TS_IDX], &FinalTxTime_64);
            thisRange.FinalTxTime = DW1000Time((int64_t)FinalTxTime_64);
    
            dist = thisRange.calculateRange(); //Decawave Formula - Use this one!
            dist_avg_formula = thisRange.calculateAvgTWRRange(); //Drift influences if Da and Db are not equal
            dist_ss = thisRange.calculateSSRange(); //Drift not compensated
            dist_ss2 = thisRange.calculateSSRange2(); //Drift not compensated, other direction
            
            char buf2[60];
            sprintf(buf2, "Packet:%d,%d,%d,%d,%d,0", recvd_resp_seq, dist_ss, dist_ss2, dist_avg_formula, dist);
            Serial.println(buf2);
    
            if (DebugUWB_L1 == 1) {
              char buf[60];
              sprintf(buf, "Packet%d  Dist:%d mm", recvd_resp_seq, dist);
              Serial.println(buf);
            }
            current_state = STATE_DIST_EST_SEND;
          }
        } else {
          current_state = STATE_RECEIVE;
          receiver(TYPICAL_RX_TIMEOUT);
        }
      }
      break;
    }
    case STATE_DIST_EST_SEND:
    {
      if (DebugUWB_L1 == 1) {
        Serial.println("State: STATE_DIST_EST_SEND");
      }
      rx_dist_msg[DST_IDX] = rx_packet[SRC_IDX];
      login_device = rx_packet[SRC_IDX];
      rx_dist_msg[SRC_IDX] = myDevID;
      rx_dist_msg[SEQ_IDX] = seq & 0xFF;
      rx_dist_msg[SEQ_IDX + 1] = seq >> 8;
      rx_dist_msg[DIST_EST_MSG_DIST_MEAS_IDX] = dist&0xFF;
      rx_dist_msg[DIST_EST_MSG_DIST_MEAS_IDX+1] = (dist >> 8)&0xFF;
      rx_dist_msg[DIST_EST_MSG_DIST_MEAS_IDX+2] = (dist >> 16)&0xFF;
      rx_dist_msg[DIST_EST_MSG_DIST_MEAS_IDX+3] = (dist >> 24)&0xFF;
      
      generic_send(rx_dist_msg, sizeof(rx_dist_msg), POLL_MSG_POLL_TX_TS_IDX, SEND_DELAY_FIXED);
      while (!sendComplete);
      sendComplete = false;
      
//      pixels.clear(); // Set all pixel colors to 'off'
//      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
//      pixels.setBrightness(50);
//      pixels.show();

      current_state=STATE_RECEIVE;
      receiver(TYPICAL_RX_TIMEOUT);
      break;
    }
  }
  if (RxTimeout == true) {
    RxTimeout = false;
    current_state = STATE_RECEIVE;
    receiver(TYPICAL_RX_TIMEOUT);
  }
}

//Other support functions
void printState() {
  switch(current_state){
    case STATE_IDLE: Serial.println("STATE_IDLE"); break;
    case STATE_POLL: Serial.println("STATE_POLL"); break;
    case STATE_RESP_EXPECTED: Serial.println("STATE_RESP_EXPECTED"); break;
    case STATE_FINAL_SEND: Serial.println("STATE_FINAL_SEND"); break;
    case STATE_TWR_DONE: Serial.println("STATE_TWR_DONE"); break;
    case STATE_RESP_SEND: Serial.println("STATE_RESP_SEND"); break;
    case STATE_FINAL_EXPECTED: Serial.println("STATE_FINAL_EXPECTED");break;
    case STATE_OTHER_POLL_EXPECTED: Serial.println("STATE_OTHER_POLL_EXPECTED");break;
    case STATE_RESP_PENDING: Serial.println("STATE_RESP_PENDING"); break;
    case STATE_DIST_EST_EXPECTED: Serial.println("STATE_DIST_EST_EXPECTED");break;
    case STATE_DIST_EST_SEND: Serial.println("STATE_DIST_EST_SEND"); break;
    case STATE_RECEIVE: Serial.println("STATE_RECEIVE"); break;
    default: Serial.print("Unknown State: "); Serial.println(current_state); break;
  }
}


#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__



int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
