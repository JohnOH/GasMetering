// Pinched from jcw as basis for my gas meter counter
// ATtiny85 measurement of peak-to-peak current at 50 Hz, w/ moving average.
// 2011-10-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
//
// Note that this "sketch" is not for the ATmega328 but for the ATtiny84/85, 
// see http://jeelabs.org/2011/10/10/ac-current-detection-works/
// Based on "GasCounter_2_Sandhurst_4_DEBUG_sleep_cpu" 20131006 JohnO

/// 8ED4070648617276696E67746F6E20476173204D6F6E69746F720000000049C9

static byte NodeID;
#define PULSES_PER_METER_INCREMENT 2    // Harvington = 2, Sandhurst = 4
#include <JeeLib.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>  
#include <OneWire.h>
#include <avr/wdt.h>
OneWire  ds(10); // DIO2 on Tiny84
#define AIO1 7 // d7
#define DIO1 8 // d8
#define AIO2 9 // d9
#define DIO2 10 // d10
#define debounce 10          // debounce time, need to factor in prescaler slowdown.
#define ACK_TIME        10  // number of milliseconds to wait for an ack
#define RADIO_SYNC_MODE 2
unsigned int RETRY_LIMIT = 9;   // maximum number of times to retry
unsigned long LastKeepAlive;
volatile unsigned long pseudoTime = 0; 
unsigned long then = 0;
unsigned int TimeOut = 416;      // Initial timeout if interrupts don't happen approx 1 hour
volatile unsigned long WaitTime; // stored time of interrupt
volatile unsigned long previous; // millis value at previous interrupt
//
/////////////////////////////////////////////////////////////////////////////////////
struct payload{                                                                    //
int tries : 4; // Count of attempts on last ACK'ed packet                          //   
int count : 4; // packet number                                                    //
unsigned int MeterIncrement; // Reported meter pulse count                         //
unsigned int interval;   // time between interrupts (ms*256)                       //
signed char temp; //Temp value                                                //
byte vcc;         //getVcc 1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250 //
} payload;                                                                         //
/////////////////////////////////////////////////////////////////////////////////////
//boolean PowerOK = true;
volatile boolean MeterChanged = false;
volatile int pulse = 3;          // Always assume meter increments at startup
unsigned int ms = 65535;         // Actually 256ms per unit with Prescaler at maximum.
//
// Watchdog timeout ISR
ISR (WDT_vect) {
  pseudoTime = pseudoTime + 8000;    /* 8 seconds between watchdog interrupts */
  bitSet (WDTCSR, WDIE);
  }
ISR (PCINT0_vect) {       // Interrupt pulses assumed clean!
  unsigned long now;      // The instant!
  now = pseudoTime;
//  if (now < WaitTime) return; /* Problems with his approach */
//  WaitTime = now + debounce;  /* When using "set_sleep_mode(SLEEP_MODE_PWR_DOWN);" */
  pulse++;
  if (pulse < PULSES_PER_METER_INCREMENT) return;
  pulse = 0;
  payload.MeterIncrement++;   // Add one to meter count
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
/*unsigned int getTemp(void)
{
  byte i;
  byte present = 0;
  byte data[12];   
  digitalWrite(AIO2, HIGH); //Power up the DS18B20
  // The DallasTemperature library can do all this work for us!
  ds.reset();
  ds.skip();  // Assume on one DS18B20 on the bus.
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
  setPrescaler(8); // Slow down to wait
  delay(3);  // Was delay(750); when preScaler was 0
  setPrescaler(0); // Back to work
  present = ds.reset();
  ds.skip();   // Assume one DS18B20 on the bus.   
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  digitalWrite(AIO2, LOW); // Power down the DS18B20
  return ((data[1] << 8) + data[0]); // return t*16
}
*/
unsigned int getAvrTemp(void)
{
  /* Tiny84
  T = k * [(ADCH << 8) | ADCL] + TOS
  */
  unsigned char high, low;
//  setPrescaler(3);  // div 8, i.e. 1 MHz
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
//  setPrescaler(0);  // div 8, i.e. 1 MHz
  low = ADCL; // Must be read first as it locks
  high = ADCH; // read second and releases the lock
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRADC); // power down the ADC
  return (((high << 8) | low) - 273);  // An approximation to centigrade
 }
//
// Initialize watchdog
void WDT_Init(void)
{
// Disable interrupts
cli();
// Reset watchdog
//wdt_reset();
// Set up WDT interrupt
WDTCSR = (1<<WDCE)|(1<<WDE);
//Start watchdog timer with 8s prescaller
WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP0)|(1<<WDP3);
//Enable global interrupts
sei();
}
unsigned int getVcc(void)
 {
  // Select bandgap as ADC input 
  bitClear(PRR, PRADC); // power up the ADC
  ADMUX = (DEFAULT << 6) | 0x21; // Delay for ~1msec at 1MHz // Tiny84 = 0x21
  ADCSRA = bit(ADPS2) | bit(ADPS1) | bit(ADPS0); // Set ADC prescaler divide by 128
  ADCSRA |= bit(ADEN); // enable the ADC
  delay(2);   // Allow ADC to settle
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
void setup () {
  delay(10000);          // Delay on startup to avoid ISP/RFM12B interference.
  setPrescaler(0);       // div 1, i.e. speed up to 8 MHz
  bitSet (PRR, PRTIM1);  // Power down Timer1
  ADCSRA &= ~ bit(ADEN); // Disable the ADC
  bitSet (PRR, PRADC);   // Power down ADC
  bitClear (ACSR, ACIE); // Disable comparitor interrupts
  bitClear (ACSR, ACD);  // Power down analogue comparitor
  payload.count = 0;
  pinMode(DIO1, INPUT);  //set the pin to input
  PCMSK0 |= (1<<PCINT2); //  tell pin change mask to listen to (DIO1)
  GIMSK  |= (1<<PCIE0);  // enable PCINT interrupt in the general interrupt mask
  WDT_Init();            // Initialise WatchDog mechanism
  sei();                 // Just to be sure
  NodeID = rf12_config();// Set up the RFM12B from eeprom
  rf12_control(0xC040);  // set low-battery level to 2.2V i.s.o. 3.1V
  rf12_sleep(RF12_SLEEP);
  payload.vcc = getVcc();// Initialise the battery voltage
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop () {
      setPrescaler(8); // div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
      while (ms < TimeOut)   // Note ms is not an accurate millsecond since using Prescaler Max & Min values. 220=min 13320=hour
      {
        ms++;
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();                         // Expecting to sleep for 8 seconds or any other interrupt!
        sleep_mode();
///// Wake up here ///////// 
        sleep_disable(); /* First thing to do is disable sleep. */

        unsigned long int now = millis();
        pseudoTime = pseudoTime + (now - then); /* Adjust time as best we can */
        then = now;

      if (MeterChanged)        // Has meter changed?
        {
          MeterChanged = false; // Reset for next pass
          payload.count++;
          setPrescaler(0); // Full Speeed! div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
          payload.temp = getAvrTemp();
          rf12_sleep(RF12_WAKEUP);
          while (!rf12_canSend())
          rf12_recvDone();
          rf12_sendStart(0, &payload, sizeof payload);
          rf12_sendWait(1);
          rf12_sleep(RF12_SLEEP);
          payload.vcc = getVcc();
          setPrescaler(8); // div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
          ms = 0;        // Restart the timeout counter
       }
   }
/////
  ms = 0;
    payload.count++;
    setPrescaler(0); // Full Speed! div 256, i.e. 1 or 8 MHz div 256 (8) = 32kHz
    payload.temp = getAvrTemp();
    int long Now = pseudoTime;
    payload.interval = Now - LastKeepAlive;
    
    payload.tries = sendACK();   // Store count of tries to get an ACK
    
    LastKeepAlive = Now;
    payload.vcc = getVcc();
}
static byte sendACK() {  // Assumed running at full speed!
  for (byte i = 1; i < RETRY_LIMIT+1; ++i) 
    {  
      rf12_sleep(RF12_WAKEUP);
      while (!rf12_canSend())
      rf12_recvDone();
      rf12_sendStart(RF12_HDR_ACK, &payload, sizeof payload, RADIO_SYNC_MODE);
      byte acked = waitForAck();
      rf12_sleep(RF12_SLEEP);
        if (acked) {
          return i;
        }
     }
  }

// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                // see http://talk.jeelabs.net/topic/811#post-4712
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | NodeID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);   // Wait a while for  the reply?
        sleep_mode();
    }
    return 0;
}

