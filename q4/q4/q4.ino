#include <avr/io.h>
#include <avr/interrupt.h>

#define CYCLES (16 * 2)
#define READCYCLES (8)
#define WRITECYCLES (8)


/*
  RST = PORTB1
  CLK = PORTB2
  DAT = PORTB3
  Button = PORTD2 (INT0)
*/

volatile uint8_t number_cycles = 0;

volatile uint8_t current_oneHundred_millis = 0;
volatile uint8_t store_oneHundred_millis = 0;
volatile uint8_t bit = 0;

volatile bool timer1_flag = false;
volatile bool last_timer1_flag = false;
volatile bool button_press_flag = false;
volatile bool button_pressed = false;
volatile bool last_button_pressed = false;


volatile bool send = false;
volatile bool read = false;

volatile bool current_low = false;
volatile bool current_high = false;
volatile bool raising_edge = false;
volatile bool falling_edge = false;




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

void init_send_data() {
  if ((current_oneHundred_millis - 0) > 5) {
    current_oneHundred_millis = 0;

    PORTB |= (1 << PORTB1);  // set RST high immediately
    DDRB |= (1 << DDB3);     // Set port DAT as ouput
    TIFR1 |= (1 << OCF1A);   // set flag=0
    TCNT1 = 0;               //reset counter1
    current_low = true;
    button_pressed = true;
    button_press_flag = false;
    falling_edge = true;  // to set DAT at bit 0
    send = true;
    timer1_flag = false;
    last_timer1_flag = false;
  }
}

void generate_PWM_PORTB2_cycles() {
  if (button_pressed) {
    if (current_low) {
      PORTB |= (1 << PORTB2);  // rasing edge
      current_low = false;
      current_high = true;
      raising_edge = true;
      falling_edge = false;
      number_cycles++;

    } else if (current_high) {
      PORTB &= ~(1 << PORTB2);  // falling edge
      current_low = true;
      current_high = false;
      raising_edge = false;
      falling_edge = true;
      number_cycles++;
    }
  }
}

void set_port(bool send_bit) {
  Serial.print("Bit ");
  Serial.print(bit);
  Serial.print(": ");
  Serial.println(send_bit);
  if (!send_bit) {
    PORTB &= ~(1 << PORTB3);
  } else {
    PORTB |= (1 << PORTB3);
  }
}

void send_data(unsigned char value) {
  if (bit >= 8 && current_high) {
    PORTB &= ~(1 << PORTB3);
    bit = 0;
    send = false;
    DDRB &= ~(1 << DDB3);  // Set port DAT as input
  }
  if (falling_edge) {  // mean that data has been send
    set_port((value >> bit) & 1);
    falling_edge = false;
    bit++;
  }
}

int main(void) {
  DDRB |= ((1 << DDB1) | (1 << DDB2) | (1 << DDB3));  // Set PORTB as output
  PORTB = 0x00;                                       //set low to all PORTB
  DDRD &= ~(1 << DDD2);                               //For button

  Serial.begin(9600);

  init_timer1();
  init_ex_interrupt0();

  sei();  //Enable the Global Interrupt Bit

  while (1) {
    if (number_cycles > CYCLES) {
      turn_off_all_GPIO();
    }

    if (timer1_flag != last_timer1_flag) {
      last_timer1_flag = timer1_flag;
      generate_PWM_PORTB2_cycles();
    }

    if (send) {
      send_data(0x87);
    }

    if (button_press_flag) {
      if (!button_pressed) {
        init_send_data();
      } else {
        button_press_flag = false;
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
