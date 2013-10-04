// Pinched from jcw as basis for my gas meter counter
// ATtiny85 measurement of peak-to-peak current at 50 Hz, w/ moving average.
// 2011-10-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
//
// Note that this "sketch" is not for the ATmega328 but for the ATtiny84/85, 
// see http://jeelabs.org/2011/10/10/ac-current-detection-works/

#include <JeeLib.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>  
#define myNodeID 17
#define ORDER 31
#define AIO1 7 // d7
#define DIO1 8 // d8
#define AIO2 9 // d9
#define DIO2 10 // d10
#define ACK_TIME        15    // number of milliseconds to wait for an ack
#define RETRY_LIMIT     10   // maximum number of times to retry
unsigned long inttime; // stored time of last interrupt
unsigned long intinterval; // interval between interrupts
byte oldintcount;
int values[ORDER];
int vmin, vmax;
//
struct payload{
byte count; // packet count
byte retries;
byte missed;
unsigned long intcnt; // interrupt count
unsigned long time; // time between interrupts (ms)
} payload;
//
byte lastTime;
static void setPrescaler (uint8_t mode) {
    cli();
    CLKPR = bit(CLKPCE);
    CLKPR = mode;
    sei();
}
ISR (PCINT0_vect) { 
  unsigned long now;
  now = millis();
  if (now < inttime) return; 
  payload.time = now - (inttime - 50);
  inttime = now + 50;
  payload.intcnt++;
}
// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
            return 1;
 //       set_sleep_mode(SLEEP_MODE_IDLE);
 //       sleep_mode();
    }
    return 0;
}
void setup () {
  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRADC); // power down the ADC
  oldintcount = 0;
  payload.intcnt = 0;
  payload.count = 0;
  payload.missed = 0;
  pinMode(DIO1, INPUT);      //set the pin to input
  digitalWrite(DIO1, HIGH); //use the internal pullup resistor
  PCMSK0 |= (1<<PCINT2); //  tell pin change mask to listen to (DIO1)
  GIMSK  |= (1<<PCIE0); // enable PCINT interrupt in the general interrupt mask
  sei();
  rf12_initialize(myNodeID, RF12_868MHZ, 212);
  rf12_sleep(RF12_SLEEP);
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  PRR = bit(PRTIM1) | bit(PRUSI) | bit(PRADC); // only keep timer 0 going
}

void loop () {
  byte nextTime;
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_mode();
  nextTime = millis() >> 4; //13; // Watchdog every 18 hours
  if (oldintcount == payload.intcnt) // Has there been a DIO1 interrupt?
  {                                  // No
    if (nextTime == lastTime)        // Watchdog interrupted!
      return;
  }
  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
  oldintcount = payload.intcnt;
  lastTime = nextTime;
  payload.count = payload.count + 1;
  bitClear(PRR, PRUSI); // enable USI h/w   // Is this needed?
//  for (byte i = 0; i < RETRY_LIMIT; ++i) {
byte acked = 0;
payload.retries = 0;
  while (!acked && payload.retries < RETRY_LIMIT) {
    rf12_sleep(RF12_WAKEUP);
    while (!rf12_canSend())
      rf12_recvDone();
      rf12_sendStart(RF12_HDR_ACK, &payload, sizeof payload);
      acked = waitForAck();
      rf12_sleep(RF12_SLEEP);             
      payload.retries++;
 }  
  if (payload.retries == RETRY_LIMIT) {
     payload.missed++;
 }
  bitSet(PRR, PRUSI); // disable USI h/w    // Is this needed?
  setPrescaler(0);   // was 15 //div 8, i.e. 1 MHz div 256 (15) = 32kHz
}
