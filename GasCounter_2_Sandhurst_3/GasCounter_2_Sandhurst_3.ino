// Pinched from jcw as basis for my gas meter counter
// ATtiny85 measurement of peak-to-peak current at 50 Hz, w/ moving average.
// 2011-10-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
//
// Note that this "sketch" is not for the ATmega328 but for the ATtiny84/85, 
// see http://jeelabs.org/2011/10/10/ac-current-detection-works/
static byte myNodeID= 15;       // node ID used for this unit
#include <JeeLib.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>  
#include <OneWire.h>
#define AIO1 7 // d7
#define DIO1 8 // d8
#define AIO2 9 // d9
#define DIO2 10 // d10
#define debounce 2 // debounce time need to factor in prescaler slowdown.
#define ACK_TIME        10  // number of milliseconds to wait for an ack
#define RADIO_SYNC_MODE 2
unsigned int RETRY_LIMIT=9;   // maximum number of times to retry
unsigned int PreviousMeterClick;
unsigned long EvenClickTime;
unsigned long LastKeepAlive;
unsigned int vmin, vmax, IntToCount;
volatile unsigned long WaitTime; // stored time of interrupt
volatile unsigned long previous; // millis value at previous interrupt
//
unsigned int MeterClick;  // There are two interrupts per meter revolution.
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte count; // packet count                                                        //
unsigned int MeterIncrement; // Reported meter pulse count                         //
unsigned int interval;   // time between interrupts (ms*256)                       //
signed int temp; //getAvrTemp value                                                //
byte vcc;         //getVcc 1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250 //
} payload;                                                                         //
/////////////////////////////////////////////////////////////////////////////////////
// byte data[12];
 // byte addr[8];
byte DS18B20Addr[8];
bool flag = true;        // So we get a request ACK packet at power on.
//bool even = true;        // Only even numbered clicks increment the counter.
bool PowerOK = true;
unsigned int ms = 65535;          // Actually 256ms per unit with Prescaler at maximum.
unsigned int TimeOut = 13200;   // Initial timeout if interrupts don't happen
//
ISR (PCINT0_vect) { 
  unsigned long now;      // The instant!
  now = millis();
  if (now < WaitTime) return; 
  WaitTime = now + debounce;
  MeterClick++;
  previous = now;
}
static void setPrescaler (uint8_t mode) {
    cli();
    CLKPR = bit(CLKPCE);
    CLKPR = mode;
    sei();
}
OneWire  ds(10); // DIO2 on Tiny84
unsigned int getTemp(void)
{
  byte i;
  byte present = 0;
  byte data[12];   
  // The DallasTemperature library can do all this work for us!
  ds.reset();
  ds.select(DS18B20Addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
  for ( i = 0; i < 750; i++) {       // Delay for 750ms
    sleep_mode(); 
  }
// we might do a ds.depower() here, but the reset will take care of it.
  present = ds.reset();
  ds.select(DS18B20Addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  return ((data[1] << 8) + data[0]) * 10 >> 4; // degrees * 10
}
/*
unsigned int getAvrTemp(void)
{
  // Tiny84
//  T = k * [(ADCH << 8) | ADCL] + TOS
  //
  unsigned char high, low;
  ADMUX = (INTERNAL << 6) | B00100010; //Selecting the ADC8 channel by writing the MUX5:0 bits in ADMUX register to “100010” enables the temperature sensor.
  CLKPR = bit(CLKPCE);  // Unlock the speed change bits
  CLKPR = 4;            // div 16 
  delayMicroseconds(62);//  
  CLKPR = bit(CLKPCE); 
  CLKPR = 0; 
  ADCSRA |= bit(ADSC); // Start a conversion for Temperature
  while (ADCSRA & bit(ADSC));
    low = ADCL; // Must be read first as it locks
    high = ADCH; // read second and releases the lock
   return (high << 8) | low;
 }
*/
unsigned int getVcc(void)
{
  // Select bandgap as ADC input 
ADMUX = (DEFAULT << 6) | 0x21; // Delay for ~1msec at 1MHz // Tiny84 = 0x21
CLKPR = bit(CLKPCE); 
CLKPR = 4; // div 16 
delayMicroseconds(62); 
CLKPR = bit(CLKPCE); 
CLKPR = 0; 
//
ADCSRA |= bit(ADSC); 
while (ADCSRA & bit(ADSC)); 
unsigned int adc = ADC; 
// Select A0 as ADC input 
ADMUX = (DEFAULT << 6) | 0; 
return (55UL * 1023UL) / (adc + 1) - 50 ; 
//return 1100UL * 1023UL / adc; 
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  // 1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
  // return (55U * 1023U) / (ADC + 1) - 50;
}
// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for  the reply?
        sleep_mode();
    }
    return 0;
}
void setup () {
  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
  ADCSRA |= bit(ADEN); // Tiny84 Enable ADC
  PreviousMeterClick = 0;
 // payload.MeterIncrement = 0;
  payload.count = 0;
  IntToCount = 0;
  pinMode(DIO1, INPUT);      //set the pin to input
  digitalWrite(DIO1, HIGH); //use the internal pullup resistor
  PCMSK0 |= (1<<PCINT2); //  tell pin change mask to listen to (DIO1)
  GIMSK  |= (1<<PCIE0); // enable PCINT interrupt in the general interrupt mask
  sei();
  rf12_initialize(myNodeID, RF12_868MHZ, 212);
  rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
  rf12_sleep(RF12_SLEEP);
  payload.vcc = getVcc(); // Initialise the battery voltage
//  set_sleep_mode(SLEEP_MODE_IDLE);   
//  sleep_mode();
  // Set up DS10B20 
//  pinMode(DIO2, INPUT);      //set the pin to input
//  digitalWrite(DIO2, HIGH); //use the internal pullup resistor
//  ds.search(DS18B20Addr); // Assume first device we find is the only one.
///////////   Debug /////////////
            rf12_sleep(RF12_WAKEUP);
            while (!rf12_canSend())
            rf12_recvDone();
            rf12_sendStart(0, &DS18B20Addr, sizeof DS18B20Addr);
            rf12_sendWait(1);
            rf12_sleep(RF12_SLEEP);
///////////////////////////////////
///////////   Debug /////////////
            rf12_sleep(RF12_WAKEUP);
            while (!rf12_canSend())
            rf12_recvDone();
            rf12_sendStart(0, &MeterClick, sizeof MeterClick);
            rf12_sendWait(1);
            rf12_sleep(RF12_SLEEP);
///////////////////////////////////
}
void loop () {
///////////   Debug /////////////
            rf12_sleep(RF12_WAKEUP);
            while (!rf12_canSend())
            rf12_recvDone();
            rf12_sendStart(0, &MeterClick, sizeof MeterClick);
            rf12_sendWait(1);
            rf12_sleep(RF12_SLEEP);
///////////////////////////////////
  unsigned long Now;      // The instant!
  setPrescaler(8); // div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
  while (ms < TimeOut)   // Note ms is not an accurate millsecond since using Prescaler Max & Min values. 220=min 13320=hour
  {
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_mode();          // Expecting to sleep for 256ms
      if (PreviousMeterClick != MeterClick)        // Interrupt has happened
      {   
        if (IntToCount > 3)  // Four interrupts required to increment the counter
        {
          IntToCount = 0;
          PreviousMeterClick = MeterClick;
          payload.MeterIncrement++;   // Add one to meter
          if (PowerOK)
          {
            payload.count++;
            Now = millis();
            payload.interval = Now - EvenClickTime;
            EvenClickTime = Now;
            setPrescaler(0); // Full Speeed! div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
            payload.temp = getTemp();
            rf12_sleep(RF12_WAKEUP);
            while (!rf12_canSend())
            rf12_recvDone();
            rf12_sendStart(0, &payload, sizeof payload);
            rf12_sendWait(1);
            rf12_sleep(RF12_SLEEP);
            payload.vcc = getVcc();
            flag = false;  // We have transmitted so no need for the "I'm alive"
          }
        }
      else 
        {
        IntToCount++;
        }
      }
    ms++;
  }
  ms = 0;
  if (flag)
  {
    payload.count++;
    setPrescaler(0); // Full Speed! div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
    payload.temp = getTemp();
    Now = millis();
    payload.interval = Now - LastKeepAlive;
    for (byte i = 0; i < RETRY_LIMIT; ++i) 
      {
        rf12_sleep(RF12_WAKEUP);
        while (!rf12_canSend())
        rf12_recvDone();
        rf12_sendStart(RF12_HDR_ACK, &payload, sizeof payload, RADIO_SYNC_MODE);
        byte acked = waitForAck();
        rf12_sleep(RF12_SLEEP);
        if (acked) {
          return;
        }
      }
    while (rf12_lowbat())
     {
       sleep_enable();
       sleep_bod_disable();
       cli(); // We don't want to wake up until batteries are replaced.
       sleep_cpu();
     } 
     LastKeepAlive = Now;
     payload.vcc = getVcc();
  }
  flag = true;   // Should send out an I'm alive packet at power up
  if (payload.vcc < 70)
    {
      TimeOut = 65535;   // Low voltage: reduce I'm alive message frequency
      RETRY_LIMIT = 0;   // Low voltage: No retransmissions
      PowerOK = false;
    }
}
