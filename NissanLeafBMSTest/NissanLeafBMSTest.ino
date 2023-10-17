#include <mcp_can.h>
#include <SPI.h>

//modified from: https://ev-olution.yolasite.com/CANa.php

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

long LCDInterval = 500;        // First and second screen update interval
long LCD3Interval = 500;       // Third screen update interval
long LCD4Interval = 2000;      // Forth screen update interval
long CANInterval = 500;        // 0x79b message repeat interval in case of unsuccessful previous attempts
long LCDMillis = 0;            // for LCD screen update
long LCD3Millis = 0;           // for LCD third screen update
long LCD4Millis = 0;           // for LCD forth screen update 
long CANMillis = 0;            // for 0x79b message repeat
float BattVolts;               // Main battery voltage
uint16_t Soc;                  // Gids
uint16_t ActSoc;               // Actual state of charge
int16_t Amp;                   
float SocPct;                  // Gids in percentage
float ActSocPct;               // Acual state of charge in percentages 
float Amps;                    // Main battery current
float kWh;                     // Energy left in main batery
float kW;                      // Power used from main battery
int BattTemp1;                 // Battery temperature sensor 1 
int BattTemp2;                 // Battery temperature sensor 2
int BattTemp3;                 // Battery temperature sensor 3
int BattTemp4;                 // Battery temperature sensor 4
int ScSt = 0;                  // Screen page state
int CANSt = 0;
uint16_t CPVmin,CPVmax,CPVdiff;// cell pair min/max/diff voltage (mV)
uint8_t RcvFrIdx;

#define MAX_SOC 281.0F
#define CAN0_INT 2             // Set INT to pin 2
MCP_CAN CAN0(9);              // Set CS to pin 9 for RP2040
#define KW_FACTOR 74.73F       // Some people prefer 80

void setup() {

  Serial.begin(112500);
  Serial.println("CANa Ver: 1.4");

  pinMode(CAN0_INT, INPUT);

  delay (2000);
  
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  // Change MCP_8MHZ to MCP_16 MHZ if your MCP2515 board is equiped with 16MHZ crystal!!!
  
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
    
    Serial.println("MCP2515 Init Successfully");
    delay(1000);
  }
  else {
    Serial.println("Error initializing MCP2515!");
    delay(5000);
  }
  CAN0.init_Mask(0, 0, 0x07ff0000);
  CAN0.init_Filt(0, 0, 0x07bb0000);
  CAN0.init_Filt(1, 0, 0x07bb0000);

  CAN0.init_Mask(1, 0, 0x07ff0000);
  CAN0.init_Filt(2, 0, 0x01db0000);
  CAN0.init_Filt(3, 0, 0x055b0000); 
  CAN0.init_Filt(4, 0, 0x05bc0000); 
  CAN0.init_Filt(5, 0, 0x05bc0000); 
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);
}

void loop() {
  if (!digitalRead(CAN0_INT)){
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if (ScSt == 0){
    /////////////////// GIDS, SOH, SOC And kWh Remaining /////////////////
      if (rxId == 0x5bc){
        Soc = (rxBuf[0] << 2) | (rxBuf[1] >> 6);
        SocPct = (Soc / MAX_SOC) * 100.0F;
        kWh = (((float)Soc) * KW_FACTOR) / 1000.0F;
      }
      /////////////////// Actual SOC /////////////////////////////////////
      else if (rxId == 0x55b) {
        ActSoc = (rxBuf[0] << 2) | (rxBuf[1] >> 6);
        ActSocPct = ActSoc / 10.0F;
      }
    }
    else if (ScSt == 1){
      ////////////////// HV Voltage, Current And Energy Used /////////////
      if (rxId == 0x1db) {
        BattVolts = (rxBuf[2] << 2) | (rxBuf[3] >> 6);
        BattVolts = BattVolts/2.0F;
        Amp = (rxBuf[0] << 3) | (rxBuf[1] >> 5);
        if (Amp & 0x0400) Amp |= 0xf800;
        Amps = -(Amp / (2.0F));
        kW = (Amps * BattVolts)/1000.0F;
      }
    }
    /////////////// HV Battery Cell Min Max Voltages//////////////////////  
    else if (ScSt == 2){
      byte data[8] = {0x02, 0x21, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff};
      byte data2[8] = {0x30, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff};
      if (CANSt == 0){
        byte sndStat = CAN0.sendMsgBuf(0x79b, 0, 8, data);
        CANMillis = millis();
        CANSt = 1;
      }
      if (rxId == 0x7bb && rxBuf[0] == 0x10) {
        byte sndStat = CAN0.sendMsgBuf(0x79b, 0, 8, data2);
      }
      else if (rxId == 0x7bb && rxBuf[0] == 0x21) {
        CPVmax = rxBuf[7];
        byte sndStat = CAN0.sendMsgBuf(0x79b, 0, 8, data2);
      }
      else if (rxId == 0x7bb && rxBuf[0] == 0x22) {
        CPVmax <<= 8;
        CPVmax |= rxBuf[1];
        CPVmin = rxBuf[2];
        CPVmin <<= 8;
        CPVmin |= rxBuf[3];
        CPVdiff = CPVmax - CPVmin;
        RcvFrIdx = 1;
      }
      if (millis() - CANMillis > CANInterval){
        CANSt = 0;
      }
    }
    ////////////// HV Battery Temperature ////////////////////////////////
    else if (ScSt == 3){
      byte data[8] = {0x02, 0x21, 0x04, 0xff, 0xff, 0xff, 0xff, 0xff};
      byte data2[8] = {0x30, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff};
      if (CANSt == 0){
        byte sndStat = CAN0.sendMsgBuf(0x79b, 0, 8, data);
        CANMillis = millis();
        CANSt = 1;
      }        
      if (rxId == 0x7bb && rxBuf[0] == 0x10) {
        BattTemp1 = rxBuf[6];
        if (BattTemp1 > 200){
          BattTemp1 = BattTemp1 - 255;
        }
        byte sndStat = CAN0.sendMsgBuf(0x79b, 0, 8, data2);
      }
      else if (rxId == 0x7bb && rxBuf[0] == 0x21) {
        BattTemp2 = rxBuf[2];
        if (BattTemp2 > 200){
          BattTemp2 = BattTemp2 - 255;
        }
        BattTemp3 = rxBuf[5];
        if (BattTemp3 > 200){
          BattTemp3 = BattTemp3 - 255;
        }
        byte sndStat = CAN0.sendMsgBuf(0x79b, 0, 8, data2);
      }
      else if (rxId == 0x7bb && rxBuf[0] == 0x22) {
        BattTemp4 = rxBuf[1];
        if (BattTemp4 > 200){
          BattTemp4 = BattTemp4 - 255;
        }
        RcvFrIdx = 1;
      }
      unsigned long currentCANMillis = millis();
      if (millis() - CANMillis > (3 * CANInterval)){
        CANSt = 0;
      }
    }
  }
  
/////////////////////Console update code /////////////////////////////////////////
  
  if (ScSt == 0 && (millis() - LCDMillis > LCDInterval)){

    LCDMillis = millis(); 

    Serial.print("Gids:");
    Serial.print(Soc);
    Serial.print(" ");

    Serial.print(SocPct);
    Serial.print("% ");
 
    Serial.print("CS:");
    Serial.print(kWh);
    Serial.print("kWh ");    
    
    Serial.print(ActSocPct);
    Serial.print("%");
    Serial.print('\n');
  }
  
  else if (ScSt == 1 && (millis() - LCDMillis > LCDInterval)){
    LCDMillis = millis(); 

    Serial.print(BattVolts);
    Serial.print("V ");

    Serial.print(Amps);
    Serial.print("A ");

    Serial.print(kW);
    Serial.print("kW\n");
  }
  
  else if (ScSt == 2 && RcvFrIdx == 1 && (millis() - LCD3Millis > LCD3Interval)){
    LCD3Millis = millis();

    Serial.print("Max:");
    Serial.print(CPVmax);
    Serial.print(" mV Diff:");

    Serial.print(" Min:");
    Serial.print(CPVmin);
    Serial.print(" mV");

    Serial.print(CPVdiff);
    Serial.print("mV\n");
    RcvFrIdx = 0;
    CANSt = 0;
  }
  
  else if (ScSt == 3 && RcvFrIdx == 1 && (millis() - LCD4Millis > LCD4Interval)){
    LCD4Millis = millis();

    Serial.print("T1:");
    Serial.print(BattTemp1);
    Serial.print("C ");

    Serial.print("T2:");
    Serial.print(BattTemp2);
    Serial.print("C ");

    Serial.print("T3:");
    Serial.print(BattTemp3);
    Serial.print("C ");

    Serial.print("T4:");
    Serial.print(BattTemp4);
    Serial.print("C\n");

    RcvFrIdx = 0;
    CANSt = 0;
  }

  //auto-increment the display type
  CANSt = 0;
  RcvFrIdx = 0;
  ScSt = 1 + ScSt;
  if (ScSt > 3){
    ScSt = 0;
  }
  delay(1000);

}