#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint16_t interval = 125;
volatile uint8_t gear_pressed_time = 1;
volatile uint8_t led_state = 1;
volatile bool reverse_sweep = false;

volatile uint16_t current_millis = 0;
volatile uint16_t store_millis_led = 0;

volatile bool gear_button_flag = false;
volatile bool direction_button_flag = false;

void init_timer1() {
  TCCR1B |= (1 << WGM12);                 // Turn on the CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));  // Set up prescaler of 64
  OCR1A = 249;                            // Set CTC compare value to 1ms at 16 MHz AVR clock , with a prescaler of 8
  TIFR1 &= ~(1 << OCF1A);                 // set flag=0
  TIMSK1 |= (1 << OCIE1A);                // Enable Output Compare A Match Interrupt
}

void init_ex_interrupt0() {
  EIFR &= ~(1 << INTF0);  // set flag=0
  EIMSK |= (1 << INT0);   // set e_interrupt0
  EICRA |= (1 << ISC01);  // set falling edge
}

void init_ex_interrupt1() {
  EIFR &= ~(1 << INTF1);  // set flag=0
  EIMSK |= (1 << INT1);   // set e_interrupt1
  EICRA |= (1 << ISC11);  // set falling edge
}

void sweep_LED() {
  if (reverse_sweep) {
    led_state--;
    if (led_state < 1) {
      led_state = 3;
    }
  } else {
    led_state++;
    if (led_state > 3) {
      led_state = 1;
    }
  }
}

// void press_gear_button() {
//   if ((current_millis - store_millis_gear) > 500) {
//     store_millis_gear = current_millis;
//     gear_pressed_time++;
//     if (gear_pressed_time > 3) {
//       gear_pressed_time = 1;
//     }
//     interval = 125 * gear_pressed_time;
//   }
// }

void press_gear_button() {
  if (current_millis > 500) {
    current_millis = 0;
    gear_pressed_time++;
    if (gear_pressed_time > 3) {
      gear_pressed_time = 1;
    }
    interval = 125 * gear_pressed_time;
  }
}

// void press_direction_button() {
//   if ((current_millis - store_millis_direction) > 500) {
//     store_millis_direction = current_millis;
//     reverse_sweep = !reverse_sweep;
//   }
// }

void press_direction_button() {
  if (current_millis > 500) {
    current_millis = 0;
    reverse_sweep = !reverse_sweep;
  }
}


int main(void) {
  DDRB |= ((1 << DDB0) | (1 << DDB1) | (1 << DDB2));  // Set LED as output
  PORTB = 0x00;                                       //turn off all LED
  DDRD &= ~((1 << DDD2) | (1 << DDD3));               //For button

  init_timer1();
  init_ex_interrupt0();
  init_ex_interrupt1();

  sei();  //Enable the Global Interrupt Bit

  while (1) {
    if ((current_millis - store_millis_led) >= interval) {
      store_millis_led = current_millis;
      sweep_LED();
    }

    switch (led_state) {
      case 1:
        PORTB = 0b00000001;
        break;
      case 2:
        PORTB = 0b00000010;
        break;
      case 3:
        PORTB = 0b00000100;
        break;
      default:
        PORTB = 0x00;
        break;
    }

    while ((current_millis - store_millis_led) < interval) {
      // make even frequency
    }

    if (gear_button_flag) {
      gear_button_flag = false;
      press_gear_button();
    }

    if (direction_button_flag) {
      direction_button_flag = false;
      press_direction_button();
    }
  }
}


ISR(TIMER1_COMPA_vect) {
  current_millis++;
}

ISR(INT0_vect) {
  //Gear
  gear_button_flag = true;
}

ISR(INT1_vect) {
  //Direction
  direction_button_flag = true;
}