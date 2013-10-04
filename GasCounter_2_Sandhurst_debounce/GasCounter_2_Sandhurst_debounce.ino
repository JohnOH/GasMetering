// Pinched from jcw as basis for my gas meter counter
// ATtiny85 measurement of peak-to-peak current at 50 Hz, w/ moving average.
// 2011-10-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
//
// Note that this "sketch" is not for the ATmega328 but for the ATtiny84/85, 
// see http://jeelabs.org/2011/10/10/ac-current-detection-works/

#include <JeeLib.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>  
#define ORDER 31
#define AIO1 7 // d7
#define DIO1 8 // d8
#define AIO2 9 // d9
#define DIO2 10 // d10
#define debounce 3 // debounce time need to factor in prescaler slowdown.
volatile unsigned long inttime; // stored time of last interrupt
unsigned long intinterval; // interval between interrupts
byte oldintcount;
unsigned int values[ORDER];
unsigned int vmin, vmax;

//
struct payload{
byte count; // packet count
unsigned int intcnt; // interrupt count
unsigned int time; // time between interrupts (ms)
signed int temp; //getAvrTemp value
byte vcc; //getVcc                   // 1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
unsigned char bounce;
} payload;
//
byte lastTime;
//
ISR (PCINT0_vect) { 
  volatile unsigned long now;
  volatile unsigned char bounce;
  now = millis();
  bounce++;
  if (now < inttime) return; 
  inttime = now + debounce;
  payload.time = now;
  payload.bounce = bounce;
  bounce = 0;
  payload.intcnt++;
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
    ADMUX = (INTERNAL << 6) | B00100010; //Selecting the ADC8 channel by writing the MUX5:0 bits in ADMUX register to “100010” enables the temperature sensor.
  CLKPR = bit(CLKPCE); 
  CLKPR = 4; // div 16 
  delayMicroseconds(62); 
  CLKPR = bit(CLKPCE); 
  CLKPR = 0; 
    ADCSRA |= bit(ADSC); // Start a conversion for Temperature
    while (ADCSRA & bit(ADSC));
    low = ADCL; // Must be read first as it locks
    high = ADCH; // read second and releases the lock
   return (high << 8) | low;
 }
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
void setup () {
  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
//  ADCSRA &= ~ bit(ADEN); // disable the ADC
//  bitSet(PRR, PRADC); // power down the ADC
// Switch to ADC8 and analog reference INTERNAL Tiny84Internal 1.1V voltage reference
  ADCSRA |= bit(ADEN); // Tiny84 Enable ADC

  oldintcount = 0;
  payload.intcnt = 0;
  payload.count = 0;
  pinMode(DIO1, INPUT);      //set the pin to input
  digitalWrite(DIO1, HIGH); //use the internal pullup resistor
  PCMSK0 |= (1<<PCINT2); //  tell pin change mask to listen to (DIO1)
  GIMSK  |= (1<<PCIE0); // enable PCINT interrupt in the general interrupt mask
  sei();
  rf12_initialize(17, RF12_868MHZ, 212);
  rf12_sleep(RF12_SLEEP);
//  ADCSRA &= ~ bit(ADEN); // disable the ADC
//  PRR = bit(PRTIM1) | bit(PRUSI) | bit(PRADC); // only keep timer 0 going
}

void loop () {
  byte nextTime;
  boolean flag;
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_mode();
// 6 = 8mins 48seconds
// 7 = 32 seconds
// 9 = 1min 3sec
// 13 = 9mins 34sec
// 14 = 34mins 38sec
  nextTime = millis() >> 14; // 13;  // Watchdog every 18 hours
  if (oldintcount == payload.intcnt) // Has there been a DIO1 interrupt?
  {                                  // No
    if (nextTime == lastTime)        // Watchdog interrupted!
      return;
  }
// Two interrupts occur per revolution of meter
if (!flag) {
  flag = true;
  setPrescaler(0); // div 1, i.e. speed up to 8 MHz
  oldintcount = payload.intcnt;
  lastTime = nextTime;
  payload.count = payload.count + 1;
//  bitClear(PRR, PRUSI); // enable USI h/w   // Is this needed?
  rf12_sleep(RF12_WAKEUP);
  payload.temp = getAvrTemp();
  while (!rf12_canSend())
    rf12_recvDone();
  rf12_sendStart(0, &payload, sizeof payload);
  rf12_sendWait(1);
  rf12_sleep(RF12_SLEEP);
//
  payload.vcc = getVcc();
//  bitSet(PRR, PRUSI); // disable USI h/w    // Is this needed?
  setPrescaler(15); // div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
 }
 else {
  flag=false;
 }
}
