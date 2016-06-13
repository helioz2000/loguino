/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifdef AVR

#include "config.h"
#include "hw.h"

#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <pins_arduino.h>

/****** Power Saving Selay ******/
void psDelay(uint32_t delay_time) {
  uint32_t timeout = millis() + delay_time;
  while(1) {
    if ((int32_t) (millis() - timeout) >= 0) {
      return;
    }
  power_save();
  }
}

/****** Pin *******/

// This is a digitalWrite() replacement that does not disrupt
// timer 2.
void pin_write(uint8_t pin, uint8_t val)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out;

  if (port == NOT_A_PIN) return;

  out = portOutputRegister(port);

  if (val == LOW) {
    uint8_t oldSREG = SREG;
    cli();
    *out &= ~bit;
    SREG = oldSREG;
  } else {
    uint8_t oldSREG = SREG;
    cli();
    *out |= bit;
    SREG = oldSREG;
  }
}

/****** Radio ******/

void radio_setup()
{
  // Configure pins
  pinMode(PTT_PIN, OUTPUT);
  pin_write(PTT_PIN, LOW);
  pinMode(AUDIO_PIN, OUTPUT);
}

void radio_ptt_on()
{
  pin_write(PTT_PIN, HIGH);
  delay(25);   // The HX1 takes 5 ms from PTT to full RF, give it 25
}

void radio_ptt_off()
{
  pin_write(PTT_PIN, LOW);
}



/****** Power ******/

void disable_bod_and_sleep()
{
  /* This will turn off brown-out detection while
   * sleeping. Unfortunately this won't work in IDLE mode.
   * Relevant info about BOD disabling: datasheet p.44
   *
   * Procedure to disable the BOD:
   *
   * 1. BODSE and BODS must be set to 1
   * 2. Turn BODSE to 0
   * 3. BODS will automatically turn 0 after 4 cycles
   *
   * The catch is that we *must* go to sleep between 2
   * and 3, ie. just before BODS turns 0.
   */
  unsigned char mcucr;

  cli();
  mcucr = MCUCR | (_BV(BODS) | _BV(BODSE));
  MCUCR = mcucr;
  MCUCR = mcucr & (~_BV(BODSE));
  sei();
  sleep_mode();    // Go to sleep
}

void power_save()
{
  /* Enter power saving mode. SLEEP_MODE_IDLE is the least saving
   * mode, but it's the only one that will keep the UART running.
   * In addition, we need timer0 to keep track of time, timer 1
   * to drive the buzzer and timer2 to keep pwm output at its rest
   * voltage.
   */

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();

  pin_write(LED_PIN, LOW);
  sleep_mode();    // Go to sleep
  pin_write(LED_PIN, HIGH);
  
  sleep_disable();  // Resume after wake up
  power_all_enable();
}



/****** Buzzer ******/

// Module constants
static const unsigned long PWM_PERIOD = F_CPU / BUZZER_FREQ;
static const unsigned long ON_CYCLES = BUZZER_FREQ * BUZZER_ON_TIME;
static const unsigned long OFF_CYCLES = BUZZER_FREQ * BUZZER_OFF_TIME;
#if BUZZER_TYPE == 0  // active buzzer
static const uint16_t DUTY_CYCLE = PWM_PERIOD;
#endif
#if BUZZER_TYPE == 1  // passive buzzer
static const uint16_t DUTY_CYCLE = PWM_PERIOD / 2;
#endif

// Module variables
static volatile bool is_buzzer_on;
static volatile bool buzzing;
static volatile unsigned long alarm;

// Exported functions
void buzzer_setup()
{
  pinMode(BUZZER_PIN, OUTPUT);
  pin_write(BUZZER_PIN, LOW);
  buzzing = false;
  is_buzzer_on = false;
  alarm = 1;

  // Top is ICR1 (WGM1=14), p.135
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12);

  // Set top to PWM_PERIOD
  ICR1 = PWM_PERIOD;

  // Enable interrupts on timer overflow
  TIMSK1 |= _BV(TOIE1);

  // Start the timer, no prescaler (CS1=1)
  TCCR1B |= _BV(CS10);
}

void buzzer_on()
{
  is_buzzer_on = true;
}

void buzzer_off()
{
  is_buzzer_on = false;
}

// Interrupt Service Routine for TIMER1. This is used to switch between the
// buzzing and quiet periods when ON_CYCLES or OFF_CYCLES are reached.
ISR (TIMER1_OVF_vect)
{
  interrupts();    // allow other interrupts (ie. modem)
  alarm--;
  if (alarm == 0) {
    buzzing = !buzzing;
    if (is_buzzer_on && buzzing) {
      switch(BUZZER_PIN) {
        case 9:
          // Non-inverting pin 9 (COM1A=2), p.135
          TCCR1A |= _BV(COM1A1);
          OCR1A = DUTY_CYCLE;
          break;
        case 10:
          // Non-inverting pin 10 (COM1B=2), p.135
          TCCR1A |= _BV(COM1B1);
          OCR1B = DUTY_CYCLE;
          break;
      }
      alarm = ON_CYCLES;
    } else {
      switch(BUZZER_PIN) {
        // Disable PWM on pin 9/10
        case 9:  TCCR1A &= ~_BV(COM1A1); break;
        case 10: TCCR1A &= ~_BV(COM1B1); break;
      }
      pin_write(BUZZER_PIN, LOW);
      alarm = OFF_CYCLES;
    }
  }
}

#endif // #ifdef AVR
