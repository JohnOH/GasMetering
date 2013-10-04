// Pinched from jcw as basis for my gas meter counter
// ATtiny85 measurement of peak-to-peak current at 50 Hz, w/ moving average.
// 2011-10-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// 2012-10-27 JohnO changed this radically to count his gas meter pulses.
//
// Note that this "sketch" is not for the ATmega328 but for the ATtiny84/85, 
// see http://jeelabs.org/2011/10/10/ac-current-detection-works/
static byte myNodeID= 16;       // node ID used for this unit
#include <JeeLib.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>  
#define AIO1 7 // d7
#define DIO1 8 // d8
#define AIO2 9 // d9
#define DIO2 10 // d10
#define debounce 10          // debounce time, need to factor in prescaler slowdown.
#define ACK_TIME        15 // 10  // number of milliseconds to wait for an ack
#define RADIO_SYNC_MODE 2
unsigned int RETRY_LIMIT=9;   // maximum number of times to retry
unsigned long LastKeepAlive;
volatile unsigned long pseudoTime = 0; 
unsigned long NextTime = 0;       // Time of transmission
unsigned long then = 0;
unsigned int TimeOut = 3600; // 6 hours    // 416 //    // Initial timeout if interrupts don't happen approx 1 hour
volatile unsigned long WaitTime; // stored time of interrupt
volatile unsigned long previous; // millis value at previous interrupt
//
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
byte count; // packet count                                                        //
unsigned int MeterIncrement; // Reported meter pulse count                         //
unsigned int interval;   // time between interrupts (ms*256)                       //
signed char temp; //Temp value                                                     //
byte vcc;         //getVcc 1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250 //
//unsigned int getCT;    // Voltage at CT                                            //
} payload;                                                                         //
/////////////////////////////////////////////////////////////////////////////////////
boolean PowerOK = true;
volatile boolean MeterChanged = false;
boolean LED1 = true;
boolean LED2 = true;
volatile int pulse = 1;          // Always assume meter increments at startup
unsigned int ms = 65535;         // Actually 256ms per unit with Prescaler at maximum.
unsigned int RestTime = 60000;   // 60 seconds
//
ISR(WDT_vect) { Sleepy::watchdogEvent(); }
//
ISR (PCINT0_vect) { 
//  LED1 = !LED1;
//  digitalWrite(AIO1, LED1);   
  unsigned long now;      // The instant!
  now = pseudoTime;
  pulse++;
  if (pulse < 2) return; // Two pulses before meter increments
  pulse = 0;
  payload.MeterIncrement++;   // Add one to meter count
  payload.interval = (now - previous) / 1000;
  MeterChanged = true;
  previous = now;
}
static void setPrescaler (uint8_t mode) {
    cli();
    CLKPR = bit(CLKPCE);
    CLKPR = mode;
    sei();
}
unsigned int getAvrTemp(void)
{
  /* Tiny84
  T = k * [(ADCH << 8) | ADCL] + TOS
  */
  unsigned char high, low;
  setPrescaler(8); // 32kHz
  bitClear(PRR, PRADC); // power up the ADC
  ADMUX = B10100010;                //Selecting the ADC8 channel by writing the MUX5:0 bits in ADMUX register to “100010” enables the temperature sensor. B10 == 1.1V Reference
// Should be ADCSRB -  ADMUX &= ~_BV( ADLAR );       // Right-adjust result - the default
//  ADMUX |= _BV( REFS1 );        // Set Ref voltage
//  ADMUX &= ~( _BV( REFS0 ) );   // to 1.1V
  ADCSRA = bit(ADPS2) | bit(ADPS1) | bit(ADPS0); // Set ADC prescaler divide by 128
  ADCSRA &= ~( _BV( ADATE ) |_BV( ADIE ) ); // Disable autotrigger, Disable Interrupt
  ADCSRA |= bit(ADEN); // enable the ADC  
  delay(2);   //Allow ADC to settle
  ADCSRA |= bit(ADSC); // Start a conversion for Temperature
  while (ADCSRA & bit(ADSC));
  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
  low = ADCL; // Must be read first as it locks
  high = ADCH; // read second and releases the lock
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRADC); // power down the ADC
  return (((high << 8) | low) - 273);  // An approximation to centigrade
 }
//
unsigned int setVcc(void)
 {
  // Select bandgap as ADC input 
  bitClear(PRR, PRADC); // power up the ADC
  ADMUX = (DEFAULT << 6) | 0x21; // Delay for ~1msec at 1MHz // Tiny84 = 0x21
  ADCSRA = bit(ADPS2) | bit(ADPS1) | bit(ADPS0); // Set ADC prescaler divide by 128
  setPrescaler(8); // 32kHz
  ADCSRA |= bit(ADEN); // enable the ADC
  delay(2);   // Allow ADC to settle
  ADCSRA |= bit(ADSC);        // Start a measurement
  setPrescaler(0); // Full speed!
}
unsigned int getVcc(void)
 {
  while (ADCSRA & bit(ADSC)); // Loop until measurement complete 
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
        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for the reply?
        sleep_mode();
    }
    return 0;
}
void setup () {
  Sleepy::loseSomeTime(1000);  // delay(1000);      // Delay on startup to avoid ISP/RFM12B interference.
  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
  bitSet (PRR, PRTIM1);  // Power down Timer1
  ADCSRA &= ~ bit(ADEN); // Disable the ADC
  bitSet (PRR, PRADC);   // Power down ADC
  bitClear (ACSR, ACIE); // Disable comparitor interrupts
  bitClear (ACSR, ACD);  // Power down analogue comparitor
  payload.count = 0;
  pinMode(DIO1, INPUT);      //set the pin to input
  PCMSK0 |= (1<<PCINT2); //  tell pin change mask to listen to (DIO1)
  GIMSK  |= (1<<PCIE0); // enable PCINT interrupt in the general interrupt mask
//  pinMode(AIO1, INPUT);
//  pinMode(AIO2, INPUT);
//  sei();                // Just to be sure
  rf12_initialize(myNodeID, RF12_868MHZ, 212);
  rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
  rf12_sleep(RF12_SLEEP);
  setVcc();               // Set up for a Vcc read
  delay(2);
  payload.vcc = getVcc(); // Initialise the battery voltage
//          digitalWrite(AIO1, LED1);   
//          digitalWrite(AIO2, LED2);   
// Tweak for underrunning RFM12B timer
        RestTime += (RestTime >> 4);  // + 1/16 Calibrate
        RestTime += (RestTime >> 6);  // + 1/64  -ditto -
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop () {
      while (ms < TimeOut)   // Note ms is not an accurate millsecond since using Prescaler Max & Min values. 220=min 13320=hour
      {
        rf12_setWatchdog(RestTime); // after ~60 s
        setPrescaler(8); // div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz // Sleep at slow speed
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();                         // Expecting to sleep for 8 seconds or any other interrupt!
//        sleep_mode();
        sleep_cpu();                            // nighty-night
 ///// Wake up here? ///////// 
        sleep_disable(); // First thing to do is disable sleep. //
        unsigned long int now = millis();
        setPrescaler(0); // Full speed
        pseudoTime = pseudoTime + (now - then); // Adjust time as best we can //
        then = now;
        if (rf12_watchdogFired()) 
        {
          pseudoTime = pseudoTime + (RestTime);    // 60 seconds between watchdog interrupts // - // Fine Tuning ? //
          rf12_setWatchdog(RestTime); // 10 seconds
          ms++;
//          LED2 = !LED2;
//          digitalWrite(AIO2, LED2);   
        }

        if (MeterChanged && NextTime < pseudoTime)      // Has meter changed? // Impose a minimum delay between transmissions
          {
//             LED1 = !LED1;
//             digitalWrite(AIO1, LED1);   
             MeterChanged = false; // Reset for next pass
             if (PowerOK)
             {
               setVcc();                // Set up for the next Vcc read
               payload.count++;
               payload.temp = getAvrTemp();
               //
               rf12_sleep(RF12_WAKEUP);
               while (!rf12_canSend())
               rf12_recvDone();
               rf12_sendStart(0, &payload, sizeof payload);
               rf12_sendWait(0);
               rf12_sleep(RF12_SLEEP);
               NextTime = pseudoTime + 59000; /* Wait a minimum time before next transmitting again */
               ms = 0;          // Restart the timeout counter
               payload.vcc = getVcc();
            }
          }
      }
/////
  ms = 0;
    payload.count++;
    payload.temp = getAvrTemp();
    int long Now = pseudoTime;
    payload.interval = (Now - LastKeepAlive) /1000;
    setVcc();                // Set up for a Vcc read
    rf12_sleep(RF12_WAKEUP);
   for (byte i = 0; i < RETRY_LIMIT; ++i) 
      {
        while (!rf12_canSend())
        rf12_recvDone();
        rf12_sendStart(RF12_HDR_ACK, &payload, sizeof payload, RADIO_SYNC_MODE);
        if (waitForAck()) {
          break;
        }
      }
      rf12_sleep(RF12_SLEEP);
      
  while (rf12_lowbat())
     {
       sleep_enable();
       sleep_bod_disable();
       PRR = bit(PRTIM0) | bit(PRTIM1) | bit(PRUSI) | bit(PRADC);
       sleep_enable();
       cli(); // We don't want to wake up until batteries are replaced.
       sleep_cpu();
      } 
//     rf12_sleep(RF12_SLEEP);
     LastKeepAlive = Now;
     payload.vcc = getVcc();
  if (payload.vcc < 61)
    {
      TimeOut = 86400 ;   // Low voltage: reduce I'm alive message frequency 24 hours
      RETRY_LIMIT = 0;      // Low voltage: No retransmissions
      PowerOK = false;
    }
}

