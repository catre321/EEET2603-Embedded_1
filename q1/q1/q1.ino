#include <avr/io.h>
#include <avr/interrupt.h>

/*
  LED = PORTB5
  BUTTON = PORTD2 (INT0)
*/

volatile bool fiveHundredHz_elapsed = false;
volatile bool fiveHz_elapsed = false;
volatile bool button_pressed = false;
volatile bool button_press_flag = false;

volatile uint8_t current_oneHundred_millis = 0;

void init_timer0() {
  TCCR0A |= (1 << WGM01);                 // Turn on the CTC mode
  TCCR0B |= ((1 << CS01) | (1 << CS00));  // Set up prescaler of 64
  OCR0A = 249;                            // Set CTC compare value to 500Hz at 16 MHz AVR clock , with a prescaler of 64
  TIFR0 &= ~(1 << OCF0A);                 // set flag=0
  TIMSK0 |= (1 << OCIE0A);                // Enable Output Compare A Match Interrupt
}

void init_timer1() {
  TCCR1B |= (1 << WGM12);   // Turn on the CTC mode
  TCCR1B |= (1 << CS12);    // Set up prescaler of 256
  OCR1A = 6249;             // Set CTC compare value to 5Hz at 16 MHz AVR clock , with a prescaler of 256
  TIFR1 &= ~(1 << OCF1A);   // set flag=0
  TIMSK1 |= (1 << OCIE1A);  // Enable Output Compare A Match Interrupt
}

void init_ex_interrupt0() {
  EIFR &= ~(1 << INTF0);  // set flag=0
  EIMSK |= (1 << INT0);   // set e_interrupt0
  EICRA |= (1 << ISC01);  // set falling edge
}


int main(void) {
  DDRB |= (1 << DDB5);   // Set LED as output
  PORTB = 0x00; // turn off all LED
  DDRD &= ~(1 << DDD2);  //For button

  init_timer0();
  init_timer1();
  init_ex_interrupt0();

  sei();  //Enable the Global Interrupt Bit

  while (1) {

    if (button_press_flag) {
      if (current_oneHundred_millis > 5) {
        current_oneHundred_millis = 0;
        button_pressed = !button_pressed;
      } 
      button_press_flag = false;
    }


    if (button_pressed) {
      if (fiveHz_elapsed) {
        PORTB ^= (1 << PORTB5);
        fiveHz_elapsed = false;
      }
    } else {
      if (fiveHundredHz_elapsed) {
        PORTB ^= (1 << PORTB5);
        fiveHundredHz_elapsed = false;
      }
    }
  }
}

ISR(TIMER0_COMPA_vect) {
  fiveHundredHz_elapsed = true;  //500Hz
}

ISR(TIMER1_COMPA_vect) {
  fiveHz_elapsed = true;  //5Hz
  current_oneHundred_millis++;
}

ISR(INT0_vect) {
  button_press_flag = true;
}