#include <JeeLib.h>
#include <avr/sleep.h>
#include <OneWire.h>
#define AIO2 9 // d9
OneWire  ds(PD7); // DIO4
static byte myNodeID= 20;       // node ID used for this unit
byte addr[8];
byte Ambient[8] = {0x28,0x9E,0x77,0x37,0x03,0x00,0x00,0xAA};
byte BoilerFeed[8] = {0x28,0x86,0x39,0x4E,0x04,0x00,0x00,0x5A};
byte CentralHeatingReturn[8] = {0x28,0x7F,0xCA,0x4D,0x4,0x0,0x0,0xDE};
byte TankCoil[8] = {0x28,0x7F,0xC6,0x4D,0x04,0x00,0x00,0xFF};
byte TankStat[8] = {0x28,0x53,0x4F,0x4E,0x04,0x00,0x00,0x84};
//
ISR(WDT_vect) { Sleepy::watchdogEvent(); }
//
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte count :8; // packet count  //
unsigned Ambient :16;
unsigned int BoilerFeed :16;
unsigned int CentralHeatingReturn :16;
unsigned int TankCoil :16;
unsigned int TankStat :16;
} payload;                                                                         //
/////////////////////////////////////////////////////////////////////////////////////
//
unsigned int getTemp(void)
{
  byte i;
  byte present = 0;
  byte data[12]; 
  
//  Serial.print("  Data = ");
//  Serial.print(present,HEX);
//  Serial.print(" ");

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
//    Serial.print(data[i], HEX);
//    Serial.print(" ");
  }
//  Serial.print(" CRC=");
//  Serial.print(OneWire::crc8(data, 8), HEX);
 // Serial.println();

  // convert the data to actual temperature

  int raw = (data[1] << 8) | data[0];
//  raw = raw << 3; // 9 bit resolution default
  raw = raw * 10;
  raw = raw >> 4;
  return (raw); // return t*10
//  return ((data[1] << 8) + data[0]); // return t*16
}
// Ten off wired DS18B20 from Alibaba
// 28 7F C6 4D 4 00 00 FF Tank Coil
// 28 7F CA 4D 4 00 00 DE Central Heating Return
// 28 53 4F 4E 4 00 00 84 Tank Stat
// 28 E8 AC 4E 4 00 00 EF
// 28 26 E6 4D 4 00 00 BF
// 28 86 39 4E 4 00 00 5A Boiler Feed
// 28 A0 68 4E 4 00 00 06
// 28 4E D3 4D 4 00 00 8B
// 28 F3 D1 4D 4 00 00 45
// 28 C9 C5 4D 4 00 00 04
//////////////////////////
void setup () {
  delay(1000);          // Delay on startup to avoid ISP/RFM12B interference.
  rf12_initialize(myNodeID, RF12_868MHZ, 212);
  rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
  rf12_sleep(RF12_SLEEP);
  Serial.begin(57600);
  Serial.println("Gas Boiler monitoring using a DS20B18");
//  Serial.println("=====================================");
//  Serial.println();
  pinMode(17, OUTPUT);      // Set the pin, AIO4 - Power the DS18B20's
  digitalWrite(17, HIGH);   // Power up the DS18B20's
/*  
/// Configure the DS20B18 ///
  ds.reset();               // Set for 9 bit measurements //
  ds.skip();                // Next command to all devices
  ds.write(0x4E);           // Write to Scratch Pad
  ds.write(0x7F);           // Set T(h)
  ds.write(0x80);           // Set T(l)
  ds.write(0x1F);           // Set Config
/// Copy config to on-chip EEProm
  ds.reset();               // Set for 9 bit measurements //
  ds.skip();                // Next command to all devices
  ds.write(0x48);           // Set Config
  delay(10);                // Wait for copy to complete
*/
  }
void loop () {
    digitalWrite(17, HIGH);   // Power up the DS18B20's
    ds.reset();
    ds.skip();              // Next command to all devices
    ds.write(0x44);         // Start all temperature conversions.
    Sleepy::loseSomeTime(100); // Wait for the data
// delay(100);
// 
    ds.reset();
    ds.select(Ambient);    
    ds.write(0xBE);         // Read Scratchpad
    payload.Ambient = getTemp();
    Serial.println(payload.Ambient);

// 
    ds.reset();
    ds.select(BoilerFeed);    
    ds.write(0xBE);         // Read Scratchpad
    payload.BoilerFeed = getTemp();
    Serial.println(payload.BoilerFeed);
// 
    ds.reset();
    ds.select(CentralHeatingReturn);    
    ds.write(0xBE);         // Read Scratchpad
    payload.CentralHeatingReturn = getTemp();
    Serial.println(payload.CentralHeatingReturn);
// 
    ds.reset();
    ds.select(TankCoil);    
    ds.write(0xBE);         // Read Scratchpad
    payload.TankCoil = getTemp();
    Serial.println(payload.TankCoil);
// 
    ds.reset();
    ds.select(TankStat);    
    ds.write(0xBE);         // Read Scratchpad
    payload.TankStat = getTemp();
    Serial.println(payload.TankStat);
    Serial.println();
    Serial.flush();

    payload.count++;

    rf12_sleep(RF12_WAKEUP);
    while (!rf12_canSend())
    rf12_recvDone();
    rf12_sendStart(0, &payload, sizeof payload);
    rf12_sendWait(2);
    rf12_sleep(RF12_SLEEP); 
    digitalWrite(17, LOW);   // Power down the DS18B20's
//    delay(6000);   
    Sleepy::loseSomeTime(60000);

}

