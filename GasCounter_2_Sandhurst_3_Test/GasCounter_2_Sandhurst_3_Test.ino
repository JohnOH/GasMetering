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
OneWire  ds(10); // DIO2 on Tiny84
#define AIO1 7 // d7
#define DIO1 8 // d8
#define AIO2 9 // d9
#define DIO2 10 // d10
#define debounce 2        // debounce time, need to factor in prescaler slowdown.
#define ACK_TIME        10  // number of milliseconds to wait for an ack
#define RADIO_SYNC_MODE 2
unsigned int RETRY_LIMIT=9;   // maximum number of times to retry
unsigned long LastKeepAlive;
volatile unsigned long WaitTime; // stored time of interrupt
volatile unsigned long previous; // millis value at previous interrupt
//
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte count; // packet count                                                        //
unsigned int MeterIncrement; // Reported meter pulse count                         //
unsigned int interval;   // time between interrupts (ms*256)                       //
signed int temp; //getAvrTemp value                                                //
byte vcc;         //getVcc 1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250 //
} payload;                                                                         //
/////////////////////////////////////////////////////////////////////////////////////
// byte DS18B20Addr[8];
bool flag = true;        // So we get a request ACK packet at power on.
bool even = false;        // Only even numbered clicks increment the counter.
bool PowerOK = true;
bool MeterChanged = false;
unsigned int ms = 65535;          // Actually 256ms per unit with Prescaler at maximum.
unsigned int TimeOut = 13200;   // Initial timeout if interrupts don't happen
//
ISR (PCINT0_vect) { 
  unsigned long now;      // The instant!
  now = millis();
  if (now < WaitTime) return; 
  WaitTime = now + debounce;
  even = !even;   // increment count every two interrupt
  if (!even) return;
  payload.MeterIncrement++;   // Add one to meter
  payload.interval = now - previous;
  MeterChanged = true;
  previous = now;
}
static void setPrescaler (uint8_t mode) {
    cli();
    CLKPR = bit(CLKPCE);
    CLKPR = mode;
    sei();
}
unsigned int getTemp(void)
{
  byte i;
  byte present = 0;
  byte data[12];   
  digitalWrite(AIO2, HIGH); //Power up the DS18B20
  // The DallasTemperature library can do all this work for us!
  ds.reset();
  ds.skip();  // Assume on one DS18B20 on the bus.
//  ds.select(DS18B20Addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
//  set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
//  for ( i = 0; i < 750; i++) {       // Delay for 750ms
//    sleep_mode(); 
//  }
setPrescaler(8); // Slow down to wait
delay(3);  // Was delay(750); when preScaler was 0
setPrescaler(0); // Back to work
// we might do a ds.depower() here, but the reset will take care of it.
  present = ds.reset();
  ds.skip();   // Assume on one DS18B20 on the bus.   
//  ds.select(DS18B20Addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
    digitalWrite(AIO2, LOW); // Power down the DS18B20
  return ((data[1] << 8) + data[0]); // return t*16
}
/*
unsigned int getTemp(void)
{
  // Tiny84
  T = k * [(ADCH << 8) | ADCL] + TOS
  //
  unsigned char high, low;
  bitClear(PRR, PRADC); // power up the ADC
  ADMUX = (INTERNAL << 6) | B00100010; // Selecting the ADC8 channel by writing the MUX5:0 bits in ADMUX register to “100010” enables the temperature sensor.
  ADCSRA |= bit(ADEN);  // enable the ADC
//  setPrescaler(4); // 
//  delayMicroseconds(62); 
//  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
  ADCSRA |= bit(ADSC);         // Start a conversion for Temperature
  while (ADCSRA & bit(ADSC));  // Loop until completed
  ADCSRA |= bit(ADSC);         // Start a conversion for Temperature
  while (ADCSRA & bit(ADSC));  // Loop until completed
  low = ADCL; // Must be read first as it locks
  high = ADCH; // read second and releases the lock   
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRADC); // power down the ADC
  return (high << 8) | low;
 }
*/
unsigned int getVcc(void)
 {
  // Select bandgap as ADC input 
  bitClear(PRR, PRADC); // power up the ADC
  ADMUX = (DEFAULT << 6) | 0x21; // Delay for ~1msec at 1MHz // Tiny84 = 0x21
  ADCSRA |= bit(ADEN); // enable the ADC
//  setPrescaler(4);
  delayMicroseconds(62); 
//  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
  ADCSRA |= bit(ADSC);        // Start a measurement
  while (ADCSRA & bit(ADSC)); // Loop until complete 
  ADCSRA |= bit(ADSC);        // Start a measurement
  while (ADCSRA & bit(ADSC)); // Loop until complete 
  unsigned int adc = ADC; 
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRADC); // power down the ADC
  return (55UL * 1023UL) / (adc + 1) - 50; 
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
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRUSI);    // 
  bitSet (PRR, PRTIM1);  // Power down Timer1
  ADCSRA &= ~ bit(ADEN); // Disable the ADC
  bitSet (PRR, PRADC);   // Power down ADC
  ADCSRA = bit(ADPS1) | bit(ADPS2); // Set ADC prescaler divider by 64 
  payload.count = 0;
  pinMode(DIO1, INPUT);      //set the pin to input
  digitalWrite(DIO1, HIGH);  //use the internal pullup resistor
  PCMSK0 |= (1<<PCINT2); //  tell pin change mask to listen to (DIO1)
  GIMSK  |= (1<<PCIE0); // enable PCINT interrupt in the general interrupt mask
  sei();
  rf12_initialize(myNodeID, RF12_868MHZ, 212);
  rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
  rf12_sleep(RF12_SLEEP);
  payload.vcc = getVcc(); // Initialise the battery voltage
  pinMode(AIO2, INPUT);      //set the pin to input - used to power the DS18B20
//  digitalWrite(AIO2, HIGH);  // use the internal pullup resistor
//  ds.search(DS18B20Addr); // Assume first device we find is the only one.
//  digitalWrite(AIO2, LOW);  // Power down.
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop () {
  setPrescaler(8); // div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
  while (ms < TimeOut)   // Note ms is not an accurate millsecond since using Prescaler Max & Min values. 220=min 13320=hour
  {
     set_sleep_mode(SLEEP_MODE_IDLE);
     sleep_mode();          // Expecting to sleep for 256ms because of preScaler
     if (MeterChanged)        // Meter has changed
     {
       MeterChanged = false; // Reset for next pass
       if (PowerOK)
       {
         payload.count++;
         setPrescaler(0); // Full Speeed! div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
         payload.temp = getTemp();
         rf12_sleep(RF12_WAKEUP);
         while (!rf12_canSend())
         rf12_recvDone();
         rf12_sendStart(0, &payload, sizeof payload);
         rf12_sendWait(1);
         rf12_sleep(RF12_SLEEP);
         payload.vcc = getVcc();
         setPrescaler(8); // div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
         flag = false;  // We have transmitted so no need for the "I'm alive"
        }
     }
     ms++;
    }
  ms = 0;
  if (flag)
  {
    payload.count++;
    setPrescaler(0); // Full Speed! div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
    payload.temp = getTemp(); // At slow clock speed if using DS18B20
    int long Now = millis();
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
       PRR = bit(PRTIM0) | bit(PRTIM1) | bit(PRUSI) | bit(PRADC);
       sleep_enable();
       cli(); // We don't want to wake up until batteries are replaced.
       sleep_cpu();
     } 
     LastKeepAlive = Now;
     payload.vcc = getVcc();
     setPrescaler(8); // div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
  }
  flag = true;   // Should have sent out an I'm alive packet at power up
  if (payload.vcc < 70)
    {
      TimeOut = 65535;   // Low voltage: reduce I'm alive message frequency
      RETRY_LIMIT = 0;   // Low voltage: No retransmissions
      PowerOK = false;
    }
}

