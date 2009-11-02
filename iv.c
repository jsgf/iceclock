/***************************************************************************
 Ice Tube Clock firmware August 13, 2009
 (c) 2009 Limor Fried / Adafruit Industries

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
****************************************************************************/


#include <avr/io.h>      
#include <string.h>
#include <avr/interrupt.h>   // Interrupts and timers
#include <util/delay.h>      // Blocking delay functions
#include <avr/pgmspace.h>    // So we can store the 'font table' in ROM
#include <avr/eeprom.h>      // Date/time/pref backup in permanent EEPROM
#include <avr/wdt.h>     // Watchdog timer to repair lockups

#include "iv.h"
#include "util.h"
#include "fonttable.h"

uint8_t region = REGION_US;

/*
 * Barrier to force compiler to make sure memory is up-to-date.  This
 * is preferable to using "volatile" because we can just resync with
 * memory when it really matters, rather than forcing the compiler to
 * do a pile of memory load/stores.
 */
#define barrier()	asm volatile("" : : : "memory")

struct timedate {
  uint8_t time_s, time_m, time_h;
  uint8_t date_m, date_d, date_y;
};

// These variables store the current time and date.
volatile struct timedate timedate;

// how loud is the speaker supposed to be?
volatile uint8_t volume;

// whether the alarm is on, going off, and alarm time
volatile uint8_t alarm_on, alarming, alarm_h, alarm_m;

// what is being displayed on the screen? (eg time, date, menu...)
volatile uint8_t displaymode;

// are we in low power sleep mode?
volatile uint8_t sleepmode = 0;

volatile uint8_t timeunknown = 0;        // MEME
volatile uint8_t restored = 0;

// Our display buffer, which is updated to show the time/date/etc
// and is multiplexed onto the tube
uint8_t display[DISPLAYSIZE]; // stores segments, not values!
uint8_t currdigit = 0;        // which digit we are currently multiplexing

// This table allow us to index between what digit we want to light up
// and what the pin number is on the MAX6921 see the .h for values.
// Stored in ROM (PROGMEM) to save RAM
const uint8_t digittable[] PROGMEM = {
  DIG_9, DIG_8, DIG_7, DIG_6, DIG_5, DIG_4, DIG_3, DIG_2, DIG_1
};

// This table allow us to index between what segment we want to light up
// and what the pin number is on the MAX6921 see the .h for values.
// Stored in ROM (PROGMEM) to save RAM
const uint8_t segmenttable[] PROGMEM = {
  SEG_H, SEG_G,  SEG_F,  SEG_E,  SEG_D,  SEG_C,  SEG_B,  SEG_A 
};

// muxdiv and MUX_DIVIDER divides down a high speed interrupt (31.25KHz)
// down so that we can refresh at about 100Hz (31.25KHz / 300)
// We refresh the entire display at 100Hz so each digit is updated
// 100Hz/DISPLAYSIZE
uint16_t muxdiv = 0;
#define MUX_DIVIDER (300 / DISPLAYSIZE)

// Likewise divides 100Hz down to 1Hz for the alarm beeping
uint16_t alarmdiv = 0;
#define ALARM_DIVIDER 100

// How long we have been snoozing
uint16_t snoozetimer = 0;

/* 
 * Idle MCU while waiting for interrupts; enables interrupts, so can
 * be used safely where interrupts are disabled (that is, there's no
 * race window between enabling interrupts and going to sleep).
 */
static inline void sleep(void)
{
  asm volatile("sei; sleep" : : : "memory");
}

// We have a non-blocking delay function, milliseconds is updated by
// an interrupt
volatile uint16_t milliseconds = 0;

static uint16_t now(void)
{
  volatile uint8_t *ms = (uint8_t *)&milliseconds;
  uint8_t h, l;

  do {
    h = ms[1];
    l = ms[0];
  } while(h != ms[1]);

  return (h << 8) | l;
}

void delayms(uint16_t ms) {
  uint16_t start = now();

  while ((now() - start) < ms)
    sleep();
}

// When the alarm is going off, pressing a button turns on snooze mode
// this sets the snoozetimer off in MAXSNOOZE seconds - which turns on
// the alarm again
void setsnooze(void) {
  //snoozetimer = eeprom_read_byte((uint8_t *)EE_SNOOZE);
  //snoozetimer *= 60; // convert minutes to seconds
  snoozetimer = MAXSNOOZE;
  DEBUGP("snooze");
  display_str("snoozing");
  displaymode = SHOW_SNOOZE;
  delayms(1000);
  displaymode = SHOW_TIME;
}

// we reset the watchdog timer 
void kickthedog(void) {
  wdt_reset();
}

static inline uint16_t time_since(uint16_t then)
{
  return now() - then;
}

/*
Button state machine:

digraph switch {
	open -> closed [label="button\npressed"];

	closed -> latched [label="held for\ndebounce\nrept=init"];
	closed -> open [label="button\nreleased"];

	sampled -> latched [label="button repeat\ntimeout\nrept=rate"];
	sampled -> open [label="button\nreleased"];

	latched -> sampled [label="state\nsampled"];
	latched -> open [label="button\nreleased"];
}
*/

#define DEBOUNCE	10	/* ms button must be pressed to be noticed */

#define REPT_INIT	800	/* ms before repeat starts */
#define REPT_RATE	60	/* ms between repeats */

#define NBUTTONS	4

/* Button latched bits */
#define BUT_MENU	0
#define BUT_SET		1
#define BUT_NEXT	2
#define BUT_ALARM	3      /* not really a button, but still needs debounce */

static uint8_t button_state;		/* state for each button */
static uint16_t button_time[NBUTTONS];	/* timestamp for current state */
static uint16_t button_lastpress;	/* timestamp last button entered LATCHED */
static uint16_t button_repeat;		/* timeout for NEXT button repeat */

/* button states */
#define BS_OPEN		0	/* not pressed */
#define BS_CLOSED	1	/* just pressed */
#define BS_LATCHED	2	/* latched, but unsampled */
#define BS_SAMPLED	3	/* latched and sampled */

#define BS_LOG_NSTATES	2
#define BS_NSTATES	(1 << BS_LOG_NSTATES)
#define BS_MASK		(BS_NSTATES - 1)

#define BSTATE(b,s)	((s) << ((b) * BS_LOG_NSTATES))

#define BOPEN(b)	BSTATE(b, BS_OPEN)
#define BCLOSED(b)	BSTATE(b, BS_CLOSED)
#define BLATCHED(b)	BSTATE(b, BS_LATCHED)
#define BSAMPLED(b)	BSTATE(b, BS_SAMPLED)

#define BMASK(b)	BSTATE(b, BS_MASK)

/*
 * We got an interrupt for a button; update state.  Expected to be
 * called with button interrupt source masked.
 * button = button number (0 - NBUTTONS)
 */
static void button_change_intr(uint8_t button, uint8_t state)
{
  uint8_t bmask = BMASK(button);
  uint8_t bstate = button_state & bmask;

  if (state) {
    /* pressed, so open->closed, anything else unaffected */
    if (bstate == BOPEN(button)) {
      bstate = BCLOSED(button);
      /* record press time for debounce */
      button_time[button] = now();
    }
  } else {
    /* button released, so anything->open */
    bstate = BOPEN(button);
  }

  /* update state */
  button_state = (button_state & ~bmask) | bstate;
}

/* Called every millisecond(ish) to update button state */
static void button_state_update(void)
{
  uint8_t i;
  uint8_t bs = button_state;
  uint16_t timeout, rept;

  for (i = 0; i < NBUTTONS; i++) {
    uint8_t s = bs & BS_MASK;
    switch (s) {
    case BS_OPEN:
    case BS_LATCHED:
      /* no time-based state changes */
      break;

    /* closed and sampled transition to latched with appropriate timeout */
    case BS_CLOSED:
      timeout = DEBOUNCE;
      rept = REPT_INIT;		/* closed->latched reset initial repeat timeout */
      goto timelatch;

    case BS_SAMPLED:
      timeout = 0;		/* no repeat for MENU and SET */
      if (i == BUT_NEXT) {
	timeout = button_repeat;
	rept = REPT_RATE;
      }
      goto timelatch;

    timelatch:
      if (timeout && time_since(button_time[i]) >= timeout) {
	s = BS_LATCHED;
	/* record latched time for repeat */
	button_time[i] = now();
	/* update repeat rate for current state */
	button_repeat = rept;
      }
      break;
    }

    /* shift out just-processed state */
    bs >>= BS_LOG_NSTATES;

    /* plonk in newly updated state */
    bs |= s << (BS_LOG_NSTATES * (NBUTTONS-1));
  }

  button_state = bs;
}

/*
 * Must be careful to make sure this is tested before now() rolls over
 * (64k ms = ~1 min)
 */
static uint8_t button_timeout(void)
{
  return time_since(button_lastpress) > (INACTIVITYTIMEOUT * 1000);
}

/* Poll the current state of button without changing it */
static uint8_t button_poll(uint8_t button)
{
  barrier();
  return (button_state & BMASK(button)) == BLATCHED(button);
}

/*
 * Given a button, return true if a button is pressed, and consume the
 * button-press.
 */
static uint8_t button_sample(uint8_t button)
{
  uint8_t ret = button_poll(button);

  if (ret) {
    /* latched -> sampled */
    button_state = (button_state & ~BMASK(button)) | BSAMPLED(button);
    barrier();

    button_lastpress = now();
    tick();
  }

  return ret;
}

// called @ (F_CPU/256) = ~30khz (31.25 khz)
SIGNAL (SIG_OVERFLOW0) {
  // allow other interrupts to go off while we're doing display updates
  sei();

  // kick the dog
  kickthedog();

  // divide down to 100Hz * digits
  muxdiv++;
  if (muxdiv < MUX_DIVIDER)
    return;
  muxdiv = 0;
  // now at 100Hz * digits

  // ok its not really 1ms but its like within 10% :)
  milliseconds++;

  // update latched and repeat state of buttons
  button_state_update();

  // Cycle through each digit in the display
  if (currdigit >= DISPLAYSIZE)
    currdigit = 0;

  // Set the current display's segments
  setdisplay(currdigit, display[currdigit]);
  // and go to the next
  currdigit++;

  // check if we should have the alarm on
  if (alarming && !snoozetimer) {
    alarmdiv++;
    if (alarmdiv > ALARM_DIVIDER) {
      alarmdiv = 0;
    } else {
      return;
    }
    // This part only gets reached at 1Hz

    // This sets the buzzer frequency
    ICR1 = 250;
    OCR1A = OCR1B = ICR1/2;

    // ok alarm is ringing!
    if (alarming & 0xF0) { // top bit indicates pulsing alarm state
      alarming &= ~0xF0;
      TCCR1B &= ~_BV(CS11); // turn buzzer off!
    } else {
      alarming |= 0xF0;
      TCCR1B |= _BV(CS11); // turn buzzer on!
    }
  }
  
}

// We use the pin change interrupts to detect when buttons are pressed

// This interrupt detects switches 1 and 3
SIGNAL(SIG_PIN_CHANGE2) {
  // allow interrupts while we're doing this
  PCMSK2 = 0;
  sei();
  // kick the dog
  kickthedog();

  button_change_intr(0, !(PIND & _BV(BUTTON1)));
  button_change_intr(2, !(PIND & _BV(BUTTON3)));

  PCMSK2 = _BV(PCINT21) | _BV(PCINT20);
}

// Just button #2
SIGNAL(SIG_PIN_CHANGE0) {
  PCMSK0 = 0;
  sei();

  button_change_intr(1, !(PINB & _BV(BUTTON2)));

  PCMSK0 = _BV(PCINT0);
}

// this goes off once a second
SIGNAL (TIMER2_OVF_vect) {
  struct timedate td;
  CLKPR = _BV(CLKPCE);  //MEME
  CLKPR = 0;

  td = timedate;

  td.time_s++;             // one second has gone by

  // a minute!
  if (td.time_s >= 60) {
    td.time_s = 0;
    td.time_m++;
  }

  // an hour...
  if (td.time_m >= 60) {
    td.time_m = 0;
    td.time_h++; 
    // lets write the time to the EEPROM
    eeprom_write_byte((uint8_t *)EE_HOUR, td.time_h);
    eeprom_write_byte((uint8_t *)EE_MIN, td.time_m);
  }

  // a day....
  if (td.time_h >= 24) {
    td.time_h = 0;
    td.date_d++;
    eeprom_write_byte((uint8_t *)EE_DAY, td.date_d);
  }

  /*
  if (! sleepmode) {
    uart_putw_dec(time_h);
    uart_putchar(':');
    uart_putw_dec(time_m);
    uart_putchar(':');
    uart_putw_dec(time_s);
    putstring_nl("");
  }
  */

  // a full month!
  // we check the leapyear and date to verify when its time to roll over months
  if ((td.date_d > 31) ||
      ((td.date_d == 31) && ((td.date_m == 4)||(td.date_m == 6)||(td.date_m == 9)||(td.date_m == 11))) ||
      ((td.date_d == 30) && (td.date_m == 2)) ||
      ((td.date_d == 29) && (td.date_m == 2) && !leapyear(2000+td.date_y))) {
    td.date_d = 1;
    td.date_m++;
    eeprom_write_byte((uint8_t *)EE_MONTH, td.date_m);
  }
  
  // HAPPY NEW YEAR!
  if (td.date_m >= 13) {
    td.date_y++;
    td.date_m = 1;
    eeprom_write_byte((uint8_t *)EE_YEAR, td.date_y);
  }
  
  timedate = td;

  // If we're in low power mode we should get out now since the display is off
  if (sleepmode)
    return;
   

  if (displaymode == SHOW_TIME) {
    if (timeunknown && (timedate.time_s % 2)) {
      display_str("        ");
    } else {
      display_time(timedate.time_h, timedate.time_m, timedate.time_s);
    }
    if (alarm_on)
      display[0] |= 0x2;
    else 
      display[0] &= ~0x2;
    
  }
  if (alarm_on && (alarm_h == timedate.time_h) && (alarm_m == timedate.time_m) && (timedate.time_s == 0)) {
    DEBUGP("alarm on!");
    alarming = 1;
    snoozetimer = 0;
  }

  if (snoozetimer) {
    snoozetimer--;
    if (snoozetimer % 2) 
      display[0] |= 0x2;
    else
      display[0] &= ~0x2;
  }
}

//Alarm Switch
SIGNAL(SIG_INTERRUPT0) {  
  EIMSK = 0;  //Disable this interrupt while we are processing it.
  uart_putchar('i');
  uint8_t x = ALARM_PIN & _BV(ALARM);
  sei();
  delayms(10); // wait for debouncing
  if (x != (ALARM_PIN & _BV(ALARM)))
    goto out;
  setalarmstate();
out:
  EIMSK = _BV(INT0);  //And reenable it before exiting.
}


SIGNAL(SIG_COMPARATOR) {
  //DEBUGP("COMP");
  if (ACSR & _BV(ACO)) {
    //DEBUGP("HIGH");
    if (!sleepmode) {
      VFDSWITCH_PORT |= _BV(VFDSWITCH); // turn off display
      VFDCLK_PORT &= ~_BV(VFDCLK) & ~_BV(VFDDATA); // no power to vfdchip
      BOOST_PORT &= ~_BV(BOOST); // pull boost fet low
      SPCR  &= ~_BV(SPE); // turn off spi
      if (restored) {
	eeprom_write_byte((uint8_t *)EE_MIN, timedate.time_m);
	eeprom_write_byte((uint8_t *)EE_SEC, timedate.time_s);
      }
      DEBUGP("z");
      TCCR0B = 0; // no boost
      volume = 0; // low power buzzer
      PCICR = 0;  // ignore buttons

      app_start();
    }
  } else {
    //DEBUGP("LOW");
    if (sleepmode) {
      if (restored) {
	eeprom_write_byte((uint8_t *)EE_MIN, timedate.time_m);
	eeprom_write_byte((uint8_t *)EE_SEC, timedate.time_s);
      }
      DEBUGP("WAKERESET"); 
      app_start();
    }
  }
}
/*********************** Main app **********/

uint32_t t;

void gotosleep(void) {
  // battery
  //if (sleepmode) //already asleep?
  //  return;
  //DEBUGP("sleeptime");
  
  sleepmode = 1;
  VFDSWITCH_PORT |= _BV(VFDSWITCH); // turn off display
  SPCR  &= ~_BV(SPE); // turn off spi
  VFDCLK_PORT &= ~_BV(VFDCLK) & ~_BV(VFDDATA); // no power to vfdchip
  BOOST_PORT &= ~_BV(BOOST); // pull boost fet low
  TCCR0B = 0; // no boost
  volume = 0; // low power buzzer
  PCICR = 0;  // ignore buttons

  // sleep time!
  //beep(3520, 1);
  //beep(1760, 1);
  //beep(880, 1);
  // turn beeper off
  PORTB &= ~_BV(SPK1) & ~_BV(SPK2); 
  
  // turn off pullups
  PORTD &= ~_BV(BUTTON1) & ~_BV(BUTTON3);
  PORTB &= ~_BV(BUTTON2);
  DDRD &= ~_BV(BUTTON1) & ~_BV(BUTTON3);
  DDRB &= ~_BV(BUTTON2);
  ALARM_PORT &= ~_BV(ALARM);
  ALARM_DDR &= ~_BV(ALARM);
  

  // reduce the clock speed
  CLKPR = _BV(CLKPCE);
  CLKPR = _BV(CLKPS3);
  
  //  PPR |= _BV(PRUSART0) | _BV(PRADC) | _BV(PRSPI) | _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRTWI);
  PORTC |= _BV(4);  // sleep signal
  sleep();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0;
  PORTC &= ~_BV(4);
}
 
 void wakeup(void) {
   if (!sleepmode)
     return;
   CLKPR = _BV(CLKPCE);
   CLKPR = 0;
   DEBUGP("waketime");
   sleepmode = 0;
   // plugged in
   // wait to verify
   _delay_ms(20);
   if (ACSR & _BV(ACO)) 
     return;
   
   // turn on pullups
   initbuttons();

   // turn on boost
   boost_init(eeprom_read_byte((uint8_t *)EE_BRIGHT));

   // turn on vfd control
   vfd_init();

   // turn on display
   VFDSWITCH_PORT &= ~_BV(VFDSWITCH); 
   VFDBLANK_PORT &= ~_BV(VFDBLANK);
   volume = eeprom_read_byte((uint8_t *)EE_VOLUME); // reset
   
   speaker_init();

   kickthedog();

   setalarmstate();

   // wake up sound
   beep(880, 1);
   beep(1760, 1);
   beep(3520, 1);

   kickthedog();
 }


void initbuttons(void) {
  DDRB =  _BV(VFDCLK) | _BV(VFDDATA) | _BV(SPK1) | _BV(SPK2);
  DDRD = _BV(BOOST) | _BV(VFDSWITCH);
  DDRC = _BV(VFDLOAD) | _BV(VFDBLANK) | _BV(4);
  PORTD = _BV(BUTTON1) | _BV(BUTTON3) | _BV(ALARM);
  PORTB = _BV(BUTTON2);

  PCICR = _BV(PCIE0) | _BV(PCIE2);
  PCMSK0 = _BV(PCINT0);
  PCMSK2 = _BV(PCINT21) | _BV(PCINT20);    

  /* Set button FSM up for current switch state */
  button_change_intr(BUT_MENU, !(PIND & _BV(BUTTON1)));
  button_change_intr(BUT_SET,  !(PINB & _BV(BUTTON2)));
  button_change_intr(BUT_NEXT, !(PIND & _BV(BUTTON3)));
  button_change_intr(BUT_ALARM, !(ALARM_PIN & _BV(ALARM)));
}



int main(void) {
  //  uint8_t i;
  uint8_t mcustate;

  // turn boost off
  TCCR0B = 0;
  BOOST_DDR |= _BV(BOOST);
  BOOST_PORT &= ~_BV(BOOST); // pull boost fet low

  // check if we were reset
  mcustate = MCUSR;
  MCUSR = 0;

  SMCR = _BV(SM1) | _BV(SM0) | _BV(SE); // sleep mode

  wdt_disable();
  // now turn it back on... 2 second time out
  //WDTCSR |= _BV(WDP0) | _BV(WDP1) | _BV(WDP2);
  //WDTCSR = _BV(WDE);
  wdt_enable(WDTO_2S);
  kickthedog();

  // we lost power at some point so lets alert the user
  // that the time may be wrong (the clock still works)
  timeunknown = 1;

  // have we read the time & date from eeprom?
  restored = 0;

  // setup uart
  uart_init(BRRL_192);
  //DEBUGP("VFD Clock");
  DEBUGP("!");

  //DEBUGP("turning on anacomp");
  // set up analog comparator
  ACSR = _BV(ACBG) | _BV(ACIE); // use bandgap, intr. on toggle!
  _delay_ms(1);
  // settle!
  if (ACSR & _BV(ACO)) {
    // hmm we should not interrupt here
    ACSR |= _BV(ACI);

    // even in low power mode, we run the clock 
    DEBUGP("clock init");
    clock_init();  

  } else {
    // we aren't in low power mode so init stuff

    // init io's
    initbuttons();
    
    VFDSWITCH_PORT &= ~_BV(VFDSWITCH);
    
    DEBUGP("turning on buttons");
    // set up button interrupts
    DEBUGP("turning on alarmsw");
    // set off an interrupt if alarm is set or unset
    EICRA = _BV(ISC00);
    EIMSK = _BV(INT0);
  
    displaymode = SHOW_TIME;
    DEBUGP("vfd init");
    vfd_init();
    
    DEBUGP("boost init");
    boost_init(eeprom_read_byte((uint8_t *)EE_BRIGHT));
    sei();

    region = eeprom_read_byte((uint8_t *)EE_REGION);
    
    DEBUGP("speaker init");
    speaker_init();

    beep(4000, 1);

    DEBUGP("clock init");
    clock_init();  

    DEBUGP("alarm init");
    setalarmstate();
  }
  DEBUGP("done");
  while (1) {
    //_delay_ms(100);
    kickthedog();
    //uart_putc_hex(ACSR);
    if (ACSR & _BV(ACO)) {
      // DEBUGP("SLEEPYTIME");
      gotosleep();
      continue;
    }
    //DEBUGP(".");

    /* If the alarm is on then any button-press will kick off snooze */
    if (alarming && (button_sample(BUT_MENU) || button_sample(BUT_SET) || button_sample(BUT_NEXT))) {
      setsnooze();
      continue;
    }

    if (button_sample(BUT_MENU)) {
      switch(displaymode) {
      case (SHOW_TIME):
	displaymode = SET_ALARM;
	display_str("set alarm");
	set_alarm();
	break;
      case (SET_ALARM):
	displaymode = SET_TIME;
	display_str("set time");
	set_time();
	timeunknown = 0;
	break;
      case (SET_TIME):
	displaymode = SET_DATE;
	display_str("set date");
	set_date();
	break;
      case (SET_DATE):
	displaymode = SET_BRIGHTNESS;
	display_str("set brit");
	set_brightness();
	break;
      case (SET_BRIGHTNESS):
	displaymode = SET_VOLUME;
	display_str("set vol ");
	set_volume();
	break;
      case (SET_VOLUME):
	displaymode = SET_REGION;
	display_str("set regn");
	set_region();
	break;
	/*
      case (SET_REGION):
	displaymode = SET_SNOOZE;
	display_str("set snoz");
	set_snooze();
	break;
	*/
      default:
	displaymode = SHOW_TIME;
      }
    } else if (button_sample(BUT_SET) || button_sample(BUT_NEXT)) {
      displaymode = NONE;
      display_date(DAY);

      kickthedog();
      delayms(1500);
      kickthedog();

      displaymode = SHOW_TIME;     
    } 
  }
}

/**************************** SUB-MENUS *****************************/

void set_alarm(void) 
{
  uint8_t mode;
  uint8_t hour, min, sec;
    
  hour = min = sec = 0;
  mode = SHOW_MENU;

  hour = alarm_h;
  min = alarm_m;
  sec = 0;
  
  while (1) {
    uint8_t highlight = 0;

    if (button_poll(BUT_MENU)) { // mode change
      return;
    }
    if (button_timeout()) {
      //timed out!
      displaymode = SHOW_TIME;     
      alarm_h = hour;
      alarm_m = min;
      eeprom_write_byte((uint8_t *)EE_ALARM_HOUR, alarm_h);    
      eeprom_write_byte((uint8_t *)EE_ALARM_MIN, alarm_m);    
      return;
    }

    if (button_sample(BUT_SET)) {
      if (mode == SHOW_MENU) {
	// ok now its selected
	mode = SET_HOUR;
	highlight = 1;
      } else if (mode == SET_HOUR) {
	mode = SET_MIN;
	highlight = 4;
      } else {
	// done!
	alarm_h = hour;
	alarm_m = min;
	eeprom_write_byte((uint8_t *)EE_ALARM_HOUR, alarm_h);    
	eeprom_write_byte((uint8_t *)EE_ALARM_MIN, alarm_m);    
	displaymode = SHOW_TIME;
	return;
      }
    }

    if (button_sample(BUT_NEXT)) {
      if (mode == SET_HOUR) {
	hour = (hour+1) % 24;
	display_alarm(hour, min);
	highlight = 1;
      }
      if (mode == SET_MIN) {
	min = (min+1) % 60;
	highlight = 4;
      }
    }

    if (highlight && mode != SHOW_MENU) {
      display_alarm(hour, min);
      display[highlight+0] |= 0x1;
      display[highlight+1] |= 0x1;
    }
  }
}

void set_time(void) 
{
  uint8_t mode;
  uint8_t hour, min, sec;
    
  hour = timedate.time_h;
  min = timedate.time_m;
  sec = timedate.time_s;
  mode = SHOW_MENU;

  while (1) {
    uint8_t highlight = 0;

    if (button_poll(BUT_MENU)) { // mode change
      return;
    }
    if (button_timeout()) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (button_sample(BUT_SET)) {
      switch (mode) {
      case SHOW_MENU:
	hour = timedate.time_h;
	min = timedate.time_m;
	sec = timedate.time_s;

	// ok now its selected
	mode = SET_HOUR;
	highlight = 1;
	break;

      case SET_HOUR:
	mode = SET_MIN;
	highlight = 4;
	break;

      case SET_MIN:
	mode = SET_SEC;
	highlight = 7;
	break;

      case SET_SEC:
	// done!
	timedate.time_h = hour;
	timedate.time_m = min;
	timedate.time_s = sec;
	displaymode = SHOW_TIME;
	return;
      }
    }

    if (button_sample(BUT_NEXT)) {
      switch (mode) {
      case SET_HOUR:
	hour = (hour+1) % 24;
	highlight = 1;
	timedate.time_h = hour;
	eeprom_write_byte((uint8_t *)EE_HOUR, timedate.time_h);
	break;

      case SET_MIN:
	min = (min+1) % 60;
	highlight = 4;
	timedate.time_m = min;	
	eeprom_write_byte((uint8_t *)EE_MIN, timedate.time_m);
	break;

      case SET_SEC:
	sec = (sec+1) % 60;
	highlight = 7;
	timedate.time_s = sec;
      }
    }

    if (highlight && mode != SHOW_MENU) {
      display_time(hour, min, sec);
      display[highlight+0] |= 0x1;
      display[highlight+1] |= 0x1;
    }
  }
}



void set_date(void) {
  uint8_t mode = SHOW_MENU;

  while (1) {
    uint8_t highlight = 0;

    if (button_timeout()) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (button_poll(BUT_MENU)) { // mode change
      return;
    }

    if (button_sample(BUT_SET)) {
      if (mode == SHOW_MENU) {
	// start!
	if (region == REGION_US) {
	  mode = SET_MONTH;
	}
	else {
	  DEBUGP("Set day");
	  mode = SET_DAY;
	}
	highlight = 1;
      } else if (((mode == SET_MONTH) && (region == REGION_US)) ||
		 ((mode == SET_DAY) && (region == REGION_EU))) {
	if (region == REGION_US)
	  mode = SET_DAY;
	else
	  mode = SET_MONTH;
	highlight = 4;
      } else if (((mode == SET_DAY) && (region == REGION_US)) ||
		 ((mode == SET_MONTH) && (region == REGION_EU))) {
	mode = SET_YEAR;
	highlight = 7;
      } else {
	displaymode = NONE;
	display_date(DATE);
	delayms(1500);
	displaymode = SHOW_TIME;
	return;
      }
    }

    if (button_sample(BUT_NEXT)) {
      switch (mode) {
      case SET_MONTH:
	timedate.date_m++;
	if (timedate.date_m >= 13)
	  timedate.date_m = 1;
	display_date(DATE);
	if (region == REGION_US) {
	  highlight = 1;
	} else {
	  highlight = 4;
	}
	eeprom_write_byte((uint8_t *)EE_MONTH, timedate.date_m);    
	break;

      case SET_DAY:
	timedate.date_d++;
	if (timedate.date_d > 31)
	  timedate.date_d = 1;
	display_date(DATE);

	if (region == REGION_EU) {
	  highlight = 1;
	} else {
	  highlight = 4;
	}
	eeprom_write_byte((uint8_t *)EE_DAY, timedate.date_d);    
	break;

      case SET_YEAR:
	timedate.date_y++;
	timedate.date_y %= 100;
	highlight = 7;
	eeprom_write_byte((uint8_t *)EE_YEAR, timedate.date_y);    
	break;
      }
    }

    if (highlight && mode != SHOW_MENU) {
      display_date(DATE);
      
      display[highlight+0] |= 0x1;
      display[highlight+1] |= 0x1;
    }
  }
}

#define EMIT_DOT	1	/* show "updating" dot */
#define EMIT_SLZ	2	/* suppress leading zero */

static void __emit_number(uint8_t *disp, uint8_t num, uint8_t flags)
{
  uint8_t extra = flags & EMIT_DOT;

  if ((flags & EMIT_SLZ) && num < 10)
    disp[0] = 0;
  else
    disp[0] = pgm_read_byte(numbertable + (num / 10)) | extra;
  disp[1] = pgm_read_byte(numbertable + (num % 10)) | extra; 
}

static void emit_number(uint8_t *disp, uint8_t num)
{
  __emit_number(disp, num, 0);
}

static void emit_number_dots(uint8_t *disp, uint8_t num)
{
  __emit_number(disp, num, EMIT_DOT);
}

static void emit_number_slz(uint8_t *disp, uint8_t num)
{
  __emit_number(disp, num, EMIT_SLZ);
}

void set_brightness(void) {
  uint8_t mode = SHOW_MENU;
  uint8_t brightness;

  brightness = eeprom_read_byte((uint8_t *)EE_BRIGHT);

  while (1) {
    if (button_timeout()) {
      //timed out!
      displaymode = SHOW_TIME;     
      eeprom_write_byte((uint8_t *)EE_BRIGHT, brightness);
      return;
    }
    if (button_poll(BUT_MENU)) { // mode change
      return;
    }
    if (button_sample(BUT_SET)) {
      if (mode == SHOW_MENU) {
	// start!
	mode = SET_BRITE;
	// display brightness
	display_str("brite ");
	emit_number_dots(&display[7], brightness);
      } else {	
	displaymode = SHOW_TIME;
	eeprom_write_byte((uint8_t *)EE_BRIGHT, brightness);
	return;
      }
    }
    if (button_sample(BUT_SET) || button_sample(BUT_NEXT)) {
      if (mode == SET_BRITE) {
	brightness += 5;
	if (brightness > 90)
	  brightness = 30;
	emit_number_dots(&display[7], brightness);

	OCR0A = ((brightness + 4) / 5) * 5;
      }
    }
  }
}

static void show_vol(uint8_t volume)
{
  char *str;

  // display volume
  if (volume) {
    display_str("vol high");
    display[5] |= 0x1;
  } else {
    display_str("vol  low");
  }
  display[6] |= 0x1;
  display[7] |= 0x1;
  display[8] |= 0x1;
}

void set_volume(void) {
  uint8_t mode = SHOW_MENU;
  uint8_t volume;

  volume = eeprom_read_byte((uint8_t *)EE_VOLUME);

  while (1) {
    if (button_timeout()) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (button_poll(BUT_MENU)) { // mode change
      return;
    }
    if (button_sample(BUT_SET)) {
      if (mode == SHOW_MENU) {
	// start!
	mode = SET_VOL;
	show_vol(volume);
      } else {	
	displaymode = SHOW_TIME;
	return;
      }
    }
    if (button_sample(BUT_NEXT)) {
      if (mode == SET_VOL) {
	volume = !volume;
	show_vol(volume);
	eeprom_write_byte((uint8_t *)EE_VOLUME, volume);
	speaker_init();
	beep(4000, 1);
      }
    }
  }
}




void set_region(void) {
  uint8_t mode = SHOW_MENU;

  region = eeprom_read_byte((uint8_t *)EE_REGION);

  while (1) {
    if (button_timeout()) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (button_poll(BUT_MENU)) { // mode change
      return;
    }
    if (button_sample(BUT_SET)) {
      if (mode == SHOW_MENU) {
	// start!
	mode = SET_REG;
	// display region
	if (region == REGION_US) {
	  display_str("usa-12hr");
	} else {
	  display_str("eur-24hr");
	}
      } else {	
	displaymode = SHOW_TIME;
	return;
      }
    }
    if (button_sample(BUT_NEXT)) {
      if (mode == SET_REG) {
	region = !region;
	if (region == REGION_US) {
	  display_str("usa-12hr");
	} else {
	  display_str("eur-24hr");
	}
	eeprom_write_byte((uint8_t *)EE_REGION, region);
      }
    }
  }
}


/*
void set_snooze(void) {
  uint8_t mode = SHOW_MENU;
  uint8_t snooze;

  timeoutcounter = INACTIVITYTIMEOUT;;  
  snooze = eeprom_read_byte((uint8_t *)EE_SNOOZE);

  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;;  
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & BUT_MENU) { // mode change
      return;
    }
    if (just_pressed & BUT_SET) {

      just_pressed = 0;
      if (mode == SHOW_MENU) {
	// start!
	mode = SET_SNOOZE;
	// display snooze
	display_str("   minut");
	display[1] = pgm_read_byte(numbertable_p + (snooze / 10)) | 0x1;
	display[2] = pgm_read_byte(numbertable_p + (snooze % 10)) | 0x1;
      } else { 
	displaymode = SHOW_TIME;
	return;
      }
    }
    if ((just_pressed & BUT_NEXT) || (pressed & BUT_NEXT)) {
      just_pressed = 0;
      if (mode == SET_SNOOZE) {
        snooze ++;
	if (snooze >= 100)
	  snooze = 0;
	display[1] = pgm_read_byte(numbertable_p + (snooze / 10)) | 0x1;
	display[2] = pgm_read_byte(numbertable_p + (snooze % 10)) | 0x1;
	eeprom_write_byte((uint8_t *)EE_SNOOZE, snooze);
      }

      if (pressed & BUT_NEXT)
	delayms(75);

    }
  }
}
*/


/**************************** RTC & ALARM *****************************/
void clock_init(void) {
  // we store the time in EEPROM when switching from power modes so its
  // reasonable to start with whats in memory
  timedate.time_h = eeprom_read_byte((uint8_t *)EE_HOUR) % 24;
  timedate.time_m = eeprom_read_byte((uint8_t *)EE_MIN) % 60;
  timedate.time_s = eeprom_read_byte((uint8_t *)EE_SEC) % 60;

  /*
    // if you're debugging, having the makefile set the right
    // time automatically will be very handy. Otherwise don't use this
  time_h = TIMEHOUR;
  time_m = TIMEMIN;
  time_s = TIMESEC + 10;
  */

  // Set up the stored alarm time and date
  alarm_m = eeprom_read_byte((uint8_t *)EE_ALARM_MIN) % 60;
  alarm_h = eeprom_read_byte((uint8_t *)EE_ALARM_HOUR) % 24;

  timedate.date_y = eeprom_read_byte((uint8_t *)EE_YEAR) % 100;
  timedate.date_m = eeprom_read_byte((uint8_t *)EE_MONTH) % 13;
  timedate.date_d = eeprom_read_byte((uint8_t *)EE_DAY) % 32;

  restored = 1;

  // Turn on the RTC by selecting the external 32khz crystal
  // 32.768 / 128 = 256 which is exactly an 8-bit timer overflow
  ASSR |= _BV(AS2); // use crystal
  TCCR2B = _BV(CS22) | _BV(CS20); // div by 128
  // We will overflow once a second, and call an interrupt

  // enable interrupt
  TIMSK2 = _BV(TOIE2);

  // enable all interrupts!
  sei();
}

// This turns on/off the alarm when the switch has been
// set. It also displays the alarm time
void setalarmstate(void) {
  if (ALARM_PIN & _BV(ALARM)) { 
    // Don't display the alarm/beep if we already have
    if  (!alarm_on) {
      // alarm on!
      alarm_on = 1;
      // reset snoozing
      snoozetimer = 0;
      // show the status on the VFD tube
      display_str("alarm on");
      // its not actually SHOW_SNOOZE but just anything but SHOW_TIME
      displaymode = SHOW_SNOOZE;
      delayms(1000);
      // show the current alarm time set
      display_alarm(alarm_h, alarm_m);
      delayms(1000);
      // after a second, go back to clock mode
      displaymode = SHOW_TIME;
    }
  } else {
    if (alarm_on) {
      // turn off the alarm
      alarm_on = 0;
      snoozetimer = 0;
      if (alarming) {
	// if the alarm is going off, we should turn it off
	// and quiet the speaker
	DEBUGP("alarm off");
	alarming = 0;
	TCCR1B &= ~_BV(CS11); // turn it off!
	PORTB |= _BV(SPK1) | _BV(SPK2);
      } 
    }
  }
}

// This will calculate leapyears, give it the year
// and it will return 1 (true) or 0 (false)
uint8_t leapyear(uint16_t y) {
  return ( (!(y % 4) && (y % 100)) || !(y % 400));
}


/**************************** SPEAKER *****************************/
// Set up the speaker to prepare for beeping!
void speaker_init(void) {

  // read the preferences for high/low volume
  volume = eeprom_read_byte((uint8_t *)EE_VOLUME);

  // We use the built-in fast PWM, 8 bit timer
  PORTB |= _BV(SPK1) | _BV(SPK2); 

  // Turn on PWM outputs for both pins
  TCCR1A = _BV(COM1B1) | _BV(COM1B0) | _BV(WGM11);
  if (volume) {
    TCCR1A |= _BV(COM1A1);
  } 
  TCCR1B = _BV(WGM13) | _BV(WGM12);

  // start at 4khz:  250 * 8 multiplier * 4000 = 8mhz
  ICR1 = 250;
  OCR1B = OCR1A = ICR1 / 2;
}

// This makes the speaker tick, it doesnt use PWM
// instead it just flicks the piezo
void tick(void) {
  TCCR1A = 0;
  TCCR1B = 0;

  // Send a pulse thru both pins, alternating
  SPK_PORT |= _BV(SPK1);
  SPK_PORT &= ~_BV(SPK2);
  delayms(10);
  SPK_PORT |= _BV(SPK2);
  SPK_PORT &= ~_BV(SPK1);
  delayms(10);
  // turn them both off
  SPK_PORT &= ~_BV(SPK1) & ~_BV(SPK2);

  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12);
}

// We can play short beeps!
void beep(uint16_t freq, uint8_t times) {
  // set the PWM output to match the desired frequency
  ICR1 = (F_CPU/8)/freq;
  // we want 50% duty cycle square wave
  OCR1A = OCR1B = ICR1/2;
   
  while (times--) {
    TCCR1B |= _BV(CS11); // turn it on!
    // beeps are 200ms long on
    _delay_ms(200);
    TCCR1B &= ~_BV(CS11); // turn it off!
    PORTB &= ~_BV(SPK1) & ~_BV(SPK2);
    // beeps are 200ms long off
    _delay_ms(200);
  }
  // turn speaker off
  PORTB &= ~_BV(SPK1) & ~_BV(SPK2);
}



/**************************** BOOST *****************************/

// We control the boost converter by changing the PWM output
// pins
void boost_init(uint8_t brightness) {
  // Set PWM value, don't set it so high that
  // we could damage the MAX chip or display
  if (brightness > 90)
    brightness = 90;

  // Or so low its not visible
  if (brightness < 30)
    brightness = 30;

  OCR0A = ((brightness + 4) / 5) * 5;

  // fast PWM, set OC0A (boost output pin) on match
  TCCR0A = _BV(WGM00) | _BV(WGM01);  

  // Use the fastest clock
  TCCR0B = _BV(CS00);
 
  TCCR0A |= _BV(COM0A1);
  TIMSK0 |= _BV(TOIE0); // turn on the interrupt for muxing
  sei();
}

/**************************** DISPLAY *****************************/

// We can display the current date!
void display_date(uint8_t style) {

  // This type is mm-dd-yy OR dd-mm-yy depending on our pref.
  if (style == DATE) {
    display[0] = 0;
    display[6] = display[3] = 0x02;     // put dashes between num

    if (region == REGION_US) {
      // mm-dd-yy
      emit_number_slz(&display[1], timedate.date_m);
      emit_number_slz(&display[4], timedate.date_d);
    } else {
      // dd-mm-yy
      emit_number_slz(&display[1], timedate.date_d);
      emit_number_slz(&display[4], timedate.date_m);
    }
    // the yy part is the same
    emit_number(&display[7], timedate.date_y);
  } else if (style == DAY) {
    // This is more "Sunday June 21" style

    uint16_t month, year;
    uint8_t dotw;

    // Calculate day of the week
    
    month = timedate.date_m;
    year = 2000 + timedate.date_y;
    if (timedate.date_m < 3)  {
      month += 12;
      year -= 1;
    }
    dotw = (timedate.date_d + (2 * month) + (6 * (month+1)/10) + year + (year/4) - (year/100) + (year/400) + 1) % 7;

    // Display the day first
    display[8] = display[7] = 0;
    switch (dotw) {
    case 0:
      display_str("sunday"); break;
    case 1:
      display_str("monday"); break;
    case 2:
      display_str("tuesday"); break;
    case 3:
      display_str("wednsday"); break;
    case 4:
      display_str("thursday"); break;
    case 5:
      display_str("friday"); break;
    case 6:
      display_str("saturday"); break;
    }
    
    // wait one seconds about
    delayms(1000);

    // Then display the month and date
    display[6] = display[5] = display[4] = 0;
    switch (timedate.date_m) {
    case 1:
      display_str("jan"); break;
    case 2:
      display_str("feb"); break;
    case 3:
      display_str("march"); break;
    case 4:
      display_str("april"); break;
    case 5:
      display_str("may"); break;
    case 6:
      display_str("june"); break;
    case 7:
      display_str("july"); break;
    case 8:
      display_str("augst"); break;
    case 9:
      display_str("sept"); break;
    case 10:
      display_str("octob"); break;
    case 11:
      display_str("novem"); break;
    case 12:
      display_str("decem"); break;
    }
    emit_number_slz(&display[7], timedate.date_d);
  }
}

// This displays a time on the clock
void display_time(uint8_t h, uint8_t m, uint8_t s) {
  
  // seconds and minutes are at the end
  emit_number(&display[7], s);
  display[6] = 0;
  emit_number(&display[4], m);
  display[3] = 0;

  // check euro (24h) or US (12h) style time
  if (region == REGION_US) {
    emit_number_slz(&display[1], ((h+11)%12)+1);

    // We use the '*' as an am/pm notice
    if (h >= 12)
      display[0] |= 0x1;  // 'pm' notice
    else 
      display[0] &= ~0x1;  // 'pm' notice
  } else {
    emit_number(&display[1], h%24);
  }
}

// Kinda like display_time but just hours and minutes
void display_alarm(uint8_t h, uint8_t m){ 
  display[8] = 0;
  display[7] = 0;
  display[6] = 0;
  emit_number(&display[4], m);
  display[3] = 0;

  // check euro or US style time
  if (region == REGION_US) {
    if (h >= 12) {
      display[0] |= 0x1;  // 'pm' notice
      display[7] = pgm_read_byte(alphatable + 'p' - 'a');
    } else {
      display[7] = pgm_read_byte(alphatable + 'a' - 'a');
      display[0] &= ~0x1;  // 'am' notice
    }
    display[8] = pgm_read_byte(alphatable + 'm' - 'a');

    emit_number_slz(&display[1], ((h+11)%12)+1);
  } else {
    emit_number(&display[1], ((h+23)%24)+1);
  }
}

// display words (menus, prompts, etc)
void display_str(char *s) {
  uint8_t i;

  // don't use the lefthand dot/slash digit
  display[0] = 0;

  // up to 8 characters
  for (i=1; i<9; i++) {
    // check for null-termination
    if (s[i-1] == 0)
      return;

    // Numbers and leters are looked up in the font table!
    if ((s[i-1] >= 'a') && (s[i-1] <= 'z')) {
      display[i] =  pgm_read_byte(alphatable + s[i-1] - 'a');
    } else if ((s[i-1] >= '0') && (s[i-1] <= '9')) {
      display[i] =  pgm_read_byte(numbertable + s[i-1] - '0');
    } else {
      display[i] = 0;      // spaces and other stuff are ignored :(
    }
  }
}

/************************* LOW LEVEL DISPLAY ************************/

// Setup SPI
void vfd_init(void) {
  SPCR  = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
}

// This changes and updates the display
// We use the digit/segment table to determine which
// pins on the MAX6921 to turn on
void setdisplay(uint8_t digit, uint8_t segments) {
  uint32_t d = 0;  // we only need 20 bits but 32 will do
  uint8_t i;

  // Set the digit selection pin
  d |= _BV(pgm_read_byte(digittable + digit));

  
  // Set the individual segments for this digit
  for (i=0; i<8; i++) {
    if (segments & _BV(i)) {
      t = 1;
      t <<= pgm_read_byte(segmenttable + i);
      d |= t;
    }
  }

  // Shift the data out to the display
  vfd_send(d);
}

// send raw data to display, its pretty straightforward. Just send 32 bits via SPI
// the bottom 20 define the segments
void vfd_send(uint32_t d) {
  // send lowest 20 bits
  cli();       // to prevent flicker we turn off interrupts
  spi_xfer(d >> 16);
  spi_xfer(d >> 8);
  spi_xfer(d);

  // latch data
  VFDLOAD_PORT |= _BV(VFDLOAD);
  VFDLOAD_PORT &= ~_BV(VFDLOAD);
  sei();
}

// Send 1 byte via SPI
void spi_xfer(uint8_t c) {

  SPDR = c;
  while (! (SPSR & _BV(SPIF)));
}

