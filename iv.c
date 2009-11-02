/***************************************************************************
 Ice Tube Clock firmware August 13, 2009
 (c) 2009 Limor Fried / Adafruit Industries
 (c) 2009 Jeremy Fitzhardinge <jeremy@goop.org>

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

static uint8_t region = REGION_US;
static uint8_t secondmode = SEC_FULL;
/*
 * Barrier to force compiler to make sure memory is up-to-date.  This
 * is preferable to using "volatile" because we can just resync with
 * memory when it really matters, rather than forcing the compiler to
 * do a pile of memory load/stores.
 */
#define barrier()	asm volatile("" : : : "memory")

static unsigned char __display_str(uint8_t *disp, const char *s);

struct timedate {
  struct time {
    uint8_t s, m, h;
  } time;
  struct date {
    uint8_t m, d, y;
  } date;
};

// These variables store the current time and date.
struct timedate timedate;
static volatile uint8_t suspend_update; /* if set, don't update */

// how loud is the speaker supposed to be?
uint8_t volume;

// whether the alarm is on, going off, and alarm time
static uint8_t alarm_on, alarming;
struct time alarm;

/* hour morning and evening start */
static uint8_t morning, evening;
static uint8_t daybrite, nightbrite;

// are we in low power sleep mode?
volatile uint8_t sleepmode = 0;

static uint8_t timeunknown = 0;        // MEME
static uint8_t restored = 0;

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
  delayms(1000);
}

// we reset the watchdog timer 
static void kickthedog(void) {
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
  uint8_t bmask;
  uint8_t bstate;

  cli();

  bmask = BMASK(button);
  bstate = button_state & bmask;

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

  sei();
}

/* Called every millisecond(ish) to update button state */
static void button_state_update(void)
{
  uint8_t i;
  uint8_t bs;
  uint16_t timeout, rept;

  cli();

  bs = button_state;
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

  sei();
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

// This will calculate leapyears, give it the year
// and it will return 1 (true) or 0 (false)
static uint8_t leapyear(uint16_t y) {
  return ( (!(y % 4) && (y % 100)) || !(y % 400));
}

static void increment_time(struct timedate *td)
{
  td->time.s++;             // one second has gone by

  // a minute!
  if (td->time.s >= 60) {
    td->time.s = 0;
    td->time.m++;
  }

  // an hour...
  if (td->time.m >= 60) {
    td->time.m = 0;
    td->time.h++; 
    // lets write the time to the EEPROM
    eeprom_write_byte((uint8_t *)EE_HOUR, td->time.h);
    eeprom_write_byte((uint8_t *)EE_MIN, td->time.m);
  }

  // a day....
  if (td->time.h >= 24) {
    td->time.h = 0;
    td->date.d++;
    eeprom_write_byte((uint8_t *)EE_DAY, td->date.d);
  }

  // a full month!
  // we check the leapyear and date to verify when its time to roll over months
  if ((td->date.d > 31) ||
      ((td->date.d == 31) && ((td->date.m == 4)||(td->date.m == 6)||(td->date.m == 9)||(td->date.m == 11))) ||
      ((td->date.d == 30) && (td->date.m == 2)) ||
      ((td->date.d == 29) && (td->date.m == 2) && !leapyear(2000+td->date.y))) {
    td->date.d = 1;
    td->date.m++;
    eeprom_write_byte((uint8_t *)EE_MONTH, td->date.m);
  }
  
  // HAPPY NEW YEAR!
  if (td->date.m >= 13) {
    td->date.y++;
    td->date.m = 1;
    eeprom_write_byte((uint8_t *)EE_YEAR, td->date.y);
  }
}

static void load_brite(void)
{
  morning = eeprom_read_byte((unsigned char *)EE_MORNINGHR);
  if (morning < 0 || morning > 12)
    morning = 6;

  evening = eeprom_read_byte((unsigned char *)EE_EVENINGHR);
  if (evening < 12 || evening > 23)
    evening = 18;

  daybrite = eeprom_read_byte((unsigned char *)EE_DAYBRITE);
  if (daybrite < BRITE_MIN || daybrite > BRITE_MAX)
    daybrite = BRITE_MAX;

  nightbrite = eeprom_read_byte((unsigned char *)EE_NIGHTBRITE);
  if (nightbrite < BRITE_MIN || nightbrite > BRITE_MAX)
    nightbrite = BRITE_MIN;
}

static void save_brite(void)
{
  eeprom_write_byte((unsigned char *)EE_MORNINGHR, morning);
  eeprom_write_byte((unsigned char *)EE_EVENINGHR, evening);
  eeprom_write_byte((unsigned char *)EE_DAYBRITE, daybrite);
  eeprom_write_byte((unsigned char *)EE_NIGHTBRITE, nightbrite);
}

static void set_brite(void)
{
  uint8_t b;

  if (alarming)
    b = (timedate.time.s % 2) ? BRITE_MIN : BRITE_MAX;
  else {
    uint8_t hour = timedate.time.h;

    b = nightbrite;
    if (hour >= morning && hour < evening)
      b = daybrite;
  }

  /* safety */
  if (b < BRITE_MIN || b > BRITE_MAX)
    b = BRITE_MIN;

  OCR0A = b;
}

/*
 * This goes off once a second, driven by the external 32.768kHz
 * crystal.  It leaves interrupts disabled so it can never itself be
 * interrupted.
 */
SIGNAL (TIMER2_OVF_vect) {
  struct timedate td;
  CLKPR = _BV(CLKPCE);  //MEME
  CLKPR = 0;

  td = timedate;

  if (!suspend_update) {
    increment_time(&td);
    timedate = td;
  }

  // If we're in low power mode we should get out now since the display is off
  if (sleepmode)
    return;
   
  if (alarm_on && (alarm.h == td.time.h) && (alarm.m == td.time.m) && (td.time.s == 0)) {
    DEBUGP("alarm on!");
    alarming = 1;
    snoozetimer = 0;
  }

  /* set brightness according to alarm state and time */
  set_brite();

  if (snoozetimer)
    snoozetimer--;
}

//Alarm Switch
SIGNAL(SIG_INTERRUPT0) {  
  EIMSK = 0;  //Disable this interrupt while we are processing it.
  uart_putchar('i');

  button_change_intr(BUT_ALARM, !(ALARM_PIN & _BV(ALARM)));

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
	eeprom_write_byte((uint8_t *)EE_MIN, timedate.time.m);
	eeprom_write_byte((uint8_t *)EE_SEC, timedate.time.s);
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
	eeprom_write_byte((uint8_t *)EE_MIN, timedate.time.m);
	eeprom_write_byte((uint8_t *)EE_SEC, timedate.time.s);
      }
      DEBUGP("WAKERESET"); 
      app_start();
    }
  }
}
/*********************** Main app **********/

#define EMIT_SLZ	1	/* suppress leading zero */

static void __emit_number(uint8_t *disp, uint8_t num, uint8_t flags)
{
  if ((flags & EMIT_SLZ) && num < 10)
    disp[0] = 0;
  else
    disp[0] = pgm_read_byte(numbertable + (num / 10));
  disp[1] = pgm_read_byte(numbertable + (num % 10)); 
}

static void emit_number(uint8_t *disp, uint8_t num)
{
  __emit_number(disp, num, 0);
}

static void emit_number_slz(uint8_t *disp, uint8_t num)
{
  __emit_number(disp, num, EMIT_SLZ);
}

struct field {
  unsigned char (*display)(unsigned char pos, unsigned char *val);
  void (*update)(unsigned char *val);
  unsigned char *val;
};

struct entry {
  const char *prompt;
  void (*get)(void);
  void (*store)(void);
};

static unsigned char show_num(unsigned char pos, unsigned char *v)
{
  emit_number(&display[pos], *v);
  return 2;
}

static unsigned char show_num_slz(unsigned char pos, unsigned char *v)
{
  emit_number_slz(&display[pos], *v);
  return 2;
}

static unsigned char show_hour(unsigned char pos, unsigned char *v)
{
  uint8_t h = *v;

  if (region == REGION_US) {
    emit_number_slz(display+pos, ((h+11) % 12) + 1);
    if (h >= 12)
      display[0] |= 0x1;	/* pm notice */
    else
      display[0] &= ~0x1;	/* am */
  } else
    emit_number(display+pos, h);
  return 2;
}

static unsigned char show_str(unsigned char pos, unsigned char *v)
{
  char str[DISPLAYSIZE + 1];

  strcpy_P(str, (char *)v);
  return __display_str(display+pos, str);
}

static unsigned char show_ampm(unsigned char pos, unsigned char *v)
{
  if (region != REGION_US)
    return 0;

  if (*v >= 12)
    return show_str(pos, (unsigned char *)PSTR("pm"));
  else
    return show_str(pos, (unsigned char *)PSTR("am"));
}

static unsigned char show_second(unsigned char pos, unsigned char *v)
{
  struct time *t = (struct time *)v;
  unsigned char s = t->s;

  switch (secondmode) {
  default:
  case SEC_FULL:
    emit_number(display+pos, s);
    return 2;

  case SEC_DIAL:
    display[pos] = (0x80 >> (s / 10)) | ((s & 1) << 1);
    return 1;

  case SEC_AMPM:
    if (region == REGION_US)
      return show_ampm(pos, &t->h);
    /* FALLTHROUGH */

  case SEC_NONE:
    display[pos] = (s & 1);
    return 1;
  }
}

static const char *dayofweek(const struct date *date)
{
  uint16_t month, year;
  uint8_t dotw;
#define DOW(dow)	static const char dow[] PROGMEM = #dow
  DOW(sunday);
  DOW(monday);
  DOW(tuesday);
  DOW(wednsday);
  DOW(thursday);
  DOW(friday);
  DOW(saturday);
#undef DOW
  static const char *days[] = {
    sunday, monday, tuesday, wednsday, thursday, friday, saturday
  };

  month = date->m;
  year = 2000 + date->y;
  if (month < 3)  {
    month += 12;
    year -= 1;
  }

  dotw = (date->d +
	  (2 * month) +
	  (6 * (month+1)/10) +
	  year + (year/4) -
	  (year/100) +
	  (year/400) + 1) % 7;

  return days[dotw];
}

static unsigned char show_dayofweek(unsigned char pos, unsigned char *v)
{
  const struct date *date = (const struct date *)v;

  return show_str(pos, (unsigned char *)dayofweek(date));
}

static const char *monthname(uint8_t month)
{
#define MON(m)	static const char m[] PROGMEM = #m
  MON(jan);
  MON(feb);
  MON(march);
  MON(april);
  MON(may);
  MON(june);
  MON(july);
  MON(augst);
  MON(sept);
  MON(octob);
  MON(novem);
  MON(decem);
#undef MON
  static const char *months[] = {
    jan, feb, march, april, may, june,
    july, augst, sept, octob, novem, decem
  };

  return months[month-1];
}

static unsigned char show_monthname(unsigned char pos, unsigned char *v)
{
  return show_str(pos, (unsigned char *)monthname(*v));
}
  
static void update_hour(unsigned char *v)
{
  if (++*v >= 24)
    *v = 0;
}

static void update_mod60(unsigned char *v)
{
  if (++*v >= 60)
    *v = 0;
}

static void update_day(unsigned char *v)
{
  if (++*v > 31)
    *v = 1;
}

static void update_month(unsigned char *v)
{
  if (++*v > 12)
    *v = 1;
}

static void update_year(unsigned char *v)
{
  if (++*v > 99)
    *v = 0;
}

static void update_brite(unsigned char *v)
{
  unsigned char new = *v;

  new += BRITE_STEP;
  if (new < BRITE_MIN)
    new = BRITE_MIN;
  if (new > BRITE_MAX)
    new = BRITE_MIN;

  OCR0A = new;

  *v = new;
}

static unsigned char show_vol(unsigned char pos, unsigned char *v)
{
  if (*v)
    return show_str(pos, (unsigned char *)PSTR("high"));
  else
    return show_str(pos, (unsigned char *)PSTR("low"));
}

static void update_toggle(unsigned char *v)
{
  *v = !*v;
}  

static void update_vol(unsigned char *v)
{
  *v = !*v;
  speaker_init();
  beep(4000, 1);
}

static unsigned char show_region(unsigned char pos, unsigned char *v)
{
  const char *ret;
  if (*v == REGION_US)
    ret = PSTR("usa-12hr");
  else
    ret = PSTR("eur-24hr");
  return show_str(pos, (unsigned char *)ret);
}

static unsigned char show_secmode(unsigned char pos, unsigned char *v)
{
  const char *ret;

  switch (*v) {
  default:
  case SEC_FULL:	ret = PSTR("full"); break;
  case SEC_DIAL:	ret = PSTR("dial"); break;
  case SEC_AMPM:	ret = PSTR("ampm"); break;
  case SEC_NONE:	ret = PSTR("none"); break;
  }
  return show_str(pos, (unsigned char *)ret);
}

static void update_secmode(unsigned char *v)
{
  if (++*v > SEC_NONE)
    *v = 0;
}

static struct menu_state {
  union {
    struct time time;
    struct date date;
    unsigned char val;
  };

  unsigned char nfields;
  struct field fields[5];
} menu_state;

static unsigned char space_P[] PROGMEM = " ";
static unsigned char dash_P[] PROGMEM = "-";

#define SPACE	  { show_str, NULL, space_P }
#define DASH	  { show_str, NULL, dash_P }

static const struct field alarm_fields[] PROGMEM = {
  { show_hour, update_hour, &alarm.h },
  DASH,
  { show_num, update_mod60, &alarm.m },
  SPACE,
  { show_ampm, NULL, &alarm.h },
};

static const struct field time_fields[] PROGMEM = {
  { show_hour, update_hour, &timedate.time.h },
  SPACE,
  { show_num, update_mod60, &timedate.time.m },
  SPACE,
  { show_second, NULL, (unsigned char *)&timedate.time },
};

static const struct field timeset_fields[] PROGMEM = {
  { show_hour, update_hour, &timedate.time.h },
  SPACE,
  { show_num, update_mod60, &timedate.time.m },
  SPACE,
  { show_num, update_mod60, &timedate.time.s },
};

static const struct field us_date_fields[] PROGMEM = {
  { show_num, update_month, &timedate.date.m },
  DASH,
  { show_num, update_day, &timedate.date.d },
  DASH,
  { show_num, update_year, &timedate.date.y },
};

static const struct field euro_date_fields[] PROGMEM = {
  { show_num, update_day, &timedate.date.d },
  DASH,
  { show_num, update_month, &timedate.date.m },
  DASH,
  { show_num, update_year, &timedate.date.y },
};

static const struct field dotw_fields[] PROGMEM = {
  { show_dayofweek, NULL, (unsigned char *)&timedate.date },
};

static const struct field monthdate_fields[] PROGMEM = {
  { show_monthname, NULL, &timedate.date.m },
  SPACE,
  { show_num_slz, NULL, &timedate.date.d },
};

static void update_morning(unsigned char *v)
{
  if (++*v >= 12)
    *v = 0;
}

unsigned char day_P[] PROGMEM = "dy ";
static const struct field day_fields[] PROGMEM = {
  { show_str, NULL, day_P },
  { show_hour, update_morning, &morning },
  SPACE,
  { show_num, update_brite, &daybrite },
};

static void update_evening(unsigned char *v)
{
  if (++*v >= 24)
    *v = 12;
}

static unsigned char night_P[] PROGMEM = "nt ";
static const struct field night_fields[] PROGMEM = {
  { show_str, NULL, night_P },
  { show_hour, update_evening, &evening },
  SPACE,
  { show_num, update_brite, &nightbrite },
};

static unsigned char vol_P[] PROGMEM = "vol ";
static const struct field vol_fields[] PROGMEM = {
  { show_str, NULL, vol_P },
  { show_vol, update_vol, &volume },
};

static const struct field region_fields[] PROGMEM = {
  { show_region, update_toggle, &region },
};

static unsigned char sec_P[] PROGMEM = "sec ";
static const struct field secmode_fields[] PROGMEM = {
  { show_str, NULL, sec_P },
  { show_secmode, update_secmode, &secondmode },
};

#define NELEM(a)	(sizeof(a) / sizeof(*a))
static void copy_fields(const struct field PROGMEM *fields, unsigned int nelem)
{
  memcpy_P(menu_state.fields, fields, nelem * sizeof(struct field));
  menu_state.nfields = nelem;
}

static void get_alarm(void)
{
  copy_fields(alarm_fields, NELEM(alarm_fields));
}

static void store_alarm(void)
{
  eeprom_write_byte((uint8_t *)EE_ALARM_HOUR, alarm.h);
  eeprom_write_byte((uint8_t *)EE_ALARM_MIN, alarm.m);
}

static void get_time(void)
{
  suspend_update = 1;
  copy_fields(timeset_fields, NELEM(timeset_fields));
  barrier();
}

static void store_time(void)
{
  timeunknown = 0;

  eeprom_write_byte((uint8_t *)EE_HOUR, timedate.time.h);
  eeprom_write_byte((uint8_t *)EE_MIN, timedate.time.m);

  suspend_update = 0;

  set_brite();
}

static void get_date(void)
{
  if (region == REGION_US) {
    copy_fields(us_date_fields, NELEM(us_date_fields));
  } else {
    copy_fields(euro_date_fields, NELEM(euro_date_fields));
  }
}

static void store_date(void)
{
  eeprom_write_byte((uint8_t *)EE_DAY, timedate.date.d);    
  eeprom_write_byte((uint8_t *)EE_MONTH, timedate.date.m);    
  eeprom_write_byte((uint8_t *)EE_YEAR, timedate.date.y);    
}

static void get_day(void)
{
  copy_fields(day_fields, NELEM(day_fields));
}

static void get_night(void)
{
  copy_fields(night_fields, NELEM(night_fields));
}

static void store_brite(void)
{
  save_brite();
  set_brite();
}

static void get_vol(void)
{
  copy_fields(vol_fields, NELEM(vol_fields));
}

static void store_vol(void)
{
   eeprom_write_byte((uint8_t *)EE_VOLUME, volume);
}

static void get_region(void)
{
  copy_fields(region_fields, NELEM(region_fields));
}

static void store_region(void)
{
  eeprom_write_byte((uint8_t *)EE_REGION, region);
}

static void get_secmode(void)
{
  copy_fields(secmode_fields, NELEM(secmode_fields));
}

static void store_secmode(void)
{
  eeprom_write_byte((uint8_t *)EE_SECONDMODE, secondmode);
}

static const struct entry mainmenu[] = {
  { "set alarm", get_alarm, store_alarm },
  { "set time", get_time, store_time },
  { "set date", get_date, store_date },
  { "day brite", get_day, store_brite },
  { "nite brit", get_night, store_brite },
  { "set vol", get_vol, store_vol },
  { "set regn", get_region, store_region },
  { "set secs", get_secmode, store_secmode },
};

static void display_entry(char highlight)
{
  uint8_t pos, i;
  const struct field *field = menu_state.fields;
  unsigned char nfields = menu_state.nfields;

  pos = 1;			/* skip indicators */
  for (i = 0; i < nfields; i++, field++) {
    uint8_t len;
    
    len = field->display(pos, field->val);

    if (highlight == i) {
      uint8_t j;

      for(j = pos; j < pos+len; j++)
	display[j] |= 0x1;
    }
    pos += len;
  }

  for (i = pos; i < DISPLAYSIZE; i++)
    display[i] = 0;
}

/* Skip to next valid input field; left unchanged if already valid. */
static uint8_t skip_to_next_input(const struct field *field, unsigned char nfields,
				  uint8_t next)
{
  while(next < nfields && field[next].update == NULL)
    next++;

  return next;
}

static void show_entry(const struct entry *entry)
{
  const struct field *field;
  uint8_t input, nfields;

  entry->get();

  nfields = menu_state.nfields;
  input = 0;

  for (;;) {
    input = skip_to_next_input(menu_state.fields, nfields, input);
    if (input >= nfields)
      break;
    field = &menu_state.fields[input];

    display_entry(input);

    for (;;) {
      kickthedog();

      if (button_timeout() || button_sample(BUT_MENU))
	goto out;

      if (button_sample(BUT_NEXT)) {
	field->update(field->val);
	break;
      }

      if (button_sample(BUT_SET)) {
	input++;
	break;
      }

      sleep();
    }
  }

out:
  entry->store();
}

static void show_menu(const struct entry *menu, int nentries)
{
  uint8_t entry = 0;

  while(entry < nentries) {
    display_str(menu->prompt);

    for (;;) {
      kickthedog();

      if (button_timeout())
	goto out;

      if (button_sample(BUT_MENU)) {
	entry++;
	menu++;
	break;
      }

      if (button_sample(BUT_SET)) {
	show_entry(menu);
	goto out;
      }

      sleep();
    }
  }
out:
  return;
}

// This displays a time on the clock
static void display_time(void)
{
  copy_fields(time_fields, NELEM(time_fields));
  display_entry(-1);
}

// Kinda like display_time but just hours and minutes
void display_alarm(void)
{
  get_alarm();
  display_entry(-1);
}

// We can display the current date!
static void display_date(uint8_t style)
{
  switch (style) {
  case DATE:
    get_date();
    display_entry(-1);
    break;

  case DAY:
    copy_fields(dotw_fields, NELEM(dotw_fields));
    display_entry(-1);
    delayms(1000);
    copy_fields(monthdate_fields, NELEM(monthdate_fields));
    display_entry(-1);
    break;
  }
}

/**************************** RTC & ALARM *****************************/
static void clock_init(void) {
  // we store the time in EEPROM when switching from power modes so its
  // reasonable to start with whats in memory
  timedate.time.h = eeprom_read_byte((uint8_t *)EE_HOUR) % 24;
  timedate.time.m = eeprom_read_byte((uint8_t *)EE_MIN) % 60;
  timedate.time.s = eeprom_read_byte((uint8_t *)EE_SEC) % 60;

  /*
    // if you're debugging, having the makefile set the right
    // time automatically will be very handy. Otherwise don't use this
  time_h = TIMEHOUR;
  time_m = TIMEMIN;
  time_s = TIMESEC + 10;
  */

  // Set up the stored alarm time and date
  alarm.m = eeprom_read_byte((uint8_t *)EE_ALARM_MIN) % 60;
  alarm.h = eeprom_read_byte((uint8_t *)EE_ALARM_HOUR) % 24;

  timedate.date.y = eeprom_read_byte((uint8_t *)EE_YEAR) % 100;
  timedate.date.m = eeprom_read_byte((uint8_t *)EE_MONTH) % 13;
  timedate.date.d = eeprom_read_byte((uint8_t *)EE_DAY) % 32;

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
 
static void wakeup(void) {
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
   boost_init();

   // turn on vfd control
   vfd_init();

   // turn on display
   VFDSWITCH_PORT &= ~_BV(VFDSWITCH); 
   VFDBLANK_PORT &= ~_BV(VFDBLANK);
   volume = eeprom_read_byte((uint8_t *)EE_VOLUME); // reset
   
   speaker_init();

   kickthedog();

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

static void ui(void)
{
  /* recheck alarm switch */
  setalarmstate();

  if (timeunknown && (timedate.time.s % 2))
    display_str("        ");
  else {
    display_time();

    if (alarm_on)
      display[0] |= 0x2;
    else 
      display[0] &= ~0x2;
  }

  if (alarming && !snoozetimer) {
    /* While alarming, any button-press will kick off snooze */
    if (button_sample(BUT_MENU) ||
	button_sample(BUT_SET) ||
	button_sample(BUT_NEXT))
      setsnooze();
  } else {
    if (button_sample(BUT_MENU))
      show_menu(mainmenu, NELEM(mainmenu));
    
    if (button_sample(BUT_SET) || button_sample(BUT_NEXT)) {
      display_date(DAY);

      delayms(1500);
    } 
  }
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
  
    load_brite();

    DEBUGP("vfd init");
    vfd_init();
    
    DEBUGP("boost init");
    boost_init();
    sei();

    region = eeprom_read_byte((uint8_t *)EE_REGION);
    secondmode = eeprom_read_byte((uint8_t *)EE_SECONDMODE);
    
    DEBUGP("speaker init");

    // read the preferences for high/low volume
    volume = eeprom_read_byte((uint8_t *)EE_VOLUME);
    speaker_init();

    beep(4000, 1);
  }
  
  DEBUGP("clock init");
  clock_init();  

  DEBUGP("done");
  while (1) {
    kickthedog();

    //uart_putc_hex(ACSR);
    if (ACSR & _BV(ACO)) {
      // DEBUGP("SLEEPYTIME");
      gotosleep();
      continue;
    }
    //DEBUGP(".");

    ui();

    /*
     * Sleep until something interesting happens (ie, an interrupt;
     * all changes are interrupt driven).  This is also a barrier, so
     * it will force all the global variables to be reloaded for the
     * next iteration.
     */
    sleep();
  }
}

// This turns on/off the alarm when the switch has been
// set. It also displays the alarm time
void setalarmstate(void) {
  uint8_t want = button_poll(BUT_ALARM);

  if (want == alarm_on)
    return;

  alarm_on = want;
  snoozetimer = 0;

  if (want) {
      // show the status on the VFD tube
      display_str("alarm on");
      // its not actually SHOW_SNOOZE but just anything but SHOW_TIME
      delayms(1000);
      // show the current alarm time set
      display_alarm();
      delayms(1000);
      // after a second, go back to clock mode
  } else {
    if (alarming) {
      // if the alarm is going off, we should turn it off
      // and quiet the speaker
      DEBUGP("alarm off");
      alarming = 0;

      /* No alarm, normal brightness */
      set_brite();

      TCCR1B &= ~_BV(CS11); // turn it off!
      PORTB |= _BV(SPK1) | _BV(SPK2);
    } 
  }
}

/**************************** SPEAKER *****************************/
// Set up the speaker to prepare for beeping!
void speaker_init(void) {

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
void boost_init(void) {
  // fast PWM, set OC0A (boost output pin) on match
  TCCR0A = _BV(WGM00) | _BV(WGM01);  

  // Use the fastest clock
  TCCR0B = _BV(CS00);
 
  TCCR0A |= _BV(COM0A1);
  TIMSK0 |= _BV(TOIE0); // turn on the interrupt for muxing

  set_brite();
  sei();
}

/**************************** DISPLAY *****************************/

// display words (menus, prompts, etc)
static unsigned char __display_str(uint8_t *disp, const char *s)
{
  unsigned char len = 0;

  while(disp < (display+DISPLAYSIZE) && *s) {
    char c = *s;

    // Numbers and leters are looked up in the font table!
    if ((c >= 'a') && (c <= 'z')) {
      *disp =  pgm_read_byte(alphatable + c - 'a');
    } else if ((c >= '0') && (c <= '9')) {
      *disp =  pgm_read_byte(numbertable + c - '0');
    } else {
      *disp = 0;      // spaces and other stuff are ignored :(
    }

    disp++;
    s++;
    len++;
  }

  return len;
}

void display_str(const char *s)
{
  uint8_t i, len;

  // don't use the lefthand dot/slash digit
  display[0] = 0;

  len = __display_str(display+1, s);
  for (i = len+1; i < DISPLAYSIZE; i++)
    display[i] = 0;
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
      uint32_t t = 1;
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
