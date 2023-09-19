#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define CYCLES (16 * 2)


/*
  RST = PORTB1
  CLK = PORTB2
  DAT = PORTB3
  Button = PORTD2 (INT0)
*/


volatile bool button_press_flag = false;
volatile bool button_pressed = false;


volatile bool send = false;
volatile bool read = false;




void init_ex_interrupt0() {
  EIFR &= ~(1 << INTF0);  // set flag=0
  EIMSK |= (1 << INT0);   // set e_interrupt0
  EICRA |= (1 << ISC01);  // set falling edge
}


void turn_off_all_GPIO() {

    PORTB = 0x00;  //set low to all PORTB
    button_pressed = false;

}

void set_port(bool send_bit) {

Serial.println(send_bit);

  if (!send_bit) {
    PORTB &= ~(1 << PORTB3);
  } else {
    PORTB |= (1 << PORTB3);
  }
}


void send_data(unsigned char value) {
  for (uint8_t bit = 0; bit < 8; bit++) {
        // PORTB &= ~(1 << PORTB3);  
    set_port((value >> bit) & 1);
    _delay_ms(100);
    // clock up, data is read by DS1302
    PORTB |= (1 << PORTB2);
    _delay_ms(100);

    if (bit == 7) {
      send = false;
      turn_off_all_GPIO();
    }

    PORTB &= ~(1 << PORTB2);  // clock down
  }
}


int main(void) {
  DDRB |= ((1 << DDB1) | (1 << DDB2) | (1 << DDB3));  // Set PORTB as output
  PORTB = 0x00;                                       //set low to all PORTB
  DDRD &= ~(1 << DDD2);  //For button

  Serial.begin(9600);


  init_ex_interrupt0();

  sei();  //Enable the Global Interrupt Bit

  while (1) {

    if (send) {
      send_data(0x83);
    }

    if (button_press_flag) {
      send = true;
      button_press_flag = false;
    }
  }
}

ISR(INT0_vect) {
  button_press_flag = true;
}
