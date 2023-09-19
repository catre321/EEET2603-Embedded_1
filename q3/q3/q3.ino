#include <avr/io.h>
#include <avr/interrupt.h>

#define CYCLES (16 * 2)

/*
  GPIO1 = PORTB1
  GPIO2 = PORTB2
  Button = PORTD2 (INT0)
*/

volatile uint8_t number_cycles = 0;

volatile uint8_t current_oneHundred_millis = 0;

volatile bool timer1_flag = false;
volatile bool last_timer1_flag = false;

volatile bool button_press_flag = false;
volatile bool button_pressed = false;

volatile bool current_low = false;
volatile bool current_high = false;


void init_timer1() {
  TCCR1B |= (1 << WGM12);   // Turn on the CTC mode
  TCCR1B |= (1 << CS12);    // Set up prescaler of 256
  OCR1A = 6250;             // Set CTC compare value to 5Hz(100ms interval) at 16 MHz AVR clock , with a prescaler of 256 (no -1 because set the counter to 0)
  TIFR1 &= ~(1 << OCF1A);   // set flag=0
  TIMSK1 |= (1 << OCIE1A);  // Enable Output Compare A Match Interrupt
}

void init_ex_interrupt0() {
  EIFR &= ~(1 << INTF0);  // set flag=0
  EIMSK |= (1 << INT0);   // set e_interrupt0
  EICRA |= (1 << ISC01);  // set falling edge
}

void turn_off_all_GPIO() {
  PORTB = 0x00;  //set low to all PORTB
  number_cycles = 0;
  button_pressed = false;
}

void init_timer1_GPIO() {
  if (current_oneHundred_millis > 5) {
    current_oneHundred_millis = 0;

    PORTB |= (1 << PORTB1);  // set PORTB1 high immediately
    TIFR1 |= (1 << OCF1A);   // set flag=0
    TCNT1 = 0;               // reset counter1
    current_low = true;
    button_pressed = true;
    timer1_flag = false;
    last_timer1_flag = false;
  }
}

void generate_PWM_PORTB2_cycles() {
  if (timer1_flag != last_timer1_flag) {
    last_timer1_flag = timer1_flag;

    if (current_low) {
      PORTB |= (1 << PORTB2);  // rasing edge
      current_low = false;
      current_high = true;
      number_cycles++;

    } else if (current_high) {
      PORTB &= ~(1 << PORTB2);  // falling edge
      current_low = true;
      current_high = false;
      number_cycles++;
    }
  }
}


int main(void) {
  DDRB |= ((1 << DDB1) | (1 << DDB2));  // set PORTB as output
  PORTB = 0x00;                         //set low to all PORTB
  DDRD &= ~(1 << DDD2);                 //For button

  init_timer1();
  init_ex_interrupt0();


  sei();  //Enable the Global Interrupt Bit

  while (1) {
    if (number_cycles > CYCLES) {
      turn_off_all_GPIO();
    }

    if (button_pressed) {
      generate_PWM_PORTB2_cycles();
    }

    if (button_press_flag) {
      button_press_flag = false;
      if (!button_pressed) {
        init_timer1_GPIO();
      }
    }
  }
}


ISR(TIMER1_COMPA_vect) {
  current_oneHundred_millis++;
  timer1_flag = !timer1_flag;
}

ISR(INT0_vect) {
  button_press_flag = true;
}