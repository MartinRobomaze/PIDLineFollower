#include "MazeCar.h"
#include <Arduino.h>
#include <avr/io.h>

/*
  Sets pin mode.
*/
void PinMode(short pin, short mode) {
  // If pin is smaller than 8 -> uses registers DDRD, PORTD.
  if (pin < 8) {
    if (mode == INPUT) {
      // Sets pin as input.
      DDRD &= !(1 << pin);
      // Disables pullup resistor.
      PORTD &= !(1 << pin);
    }

    else if (mode == INPUT_PULLUP) {
      // Sets pin as input.
      DDRD &= !(1 << pin);
      // Enables pullup resistor.
      PORTD |= 1 << pin;
    }

    else if (mode == OUTPUT) {
      // Sets pin as output.
      DDRD |= 1 << pin;
      // Writes logical zero to the pin.
      PORTD &= !(1 << pin);
    }
  }

  // Else if pin is smaller than 14 -> uses registers DDRB, PORTB.
  else if (pin < 14) {
    // Separates 3 LSB from pin.
    pin = pin & 0b00000111;
    if (mode == INPUT) {
      // Sets pin as input.
      DDRB &= !(1 << pin);
      // Disables pullup resistor.
      PORTB &= !(1 << pin);
    }

    else if (mode == INPUT_PULLUP) {
      // Sets pin as input.
      DDRB &= !(1 << pin);
      // Enables pullup resistor.
      PORTB |= 1 << pin;
    }

    else if (mode == OUTPUT) {
      // Sets pin as output.
      DDRB |= 1 << pin;
      // Writes logical zero to the pin.
      PORTB &= !(1 << pin);
    }

  }
  // Else if pin is smaller than 22 -> uses registers DDRC, PORTC.
  else if (pin < 22) {
    // Separates 3 LSB from pin.
    pin = (pin - 2) & 0b00000111;

    if (mode == INPUT) {
      // Sets pin as input.
      DDRC &= !(1 << pin);
      // Disables pullup resistor.
      PORTC &= !(1 << pin);
    }

    else if (mode == INPUT_PULLUP) {
      // Sets pin as input.
      DDRC &= !(1 << pin);
      // Enables pullup resistor.
      PORTC |= 1 << pin;
    }

    else if (mode == OUTPUT) {
      // Sets pin as output.
      DDRC |= 1 << pin;
      // Writes logical zero to the pin.
      PORTC &= !(1 << pin);
    }
  }
}

void PWMSetup(short pin) {
  // If pin is 5 or 6, use timer 0.
  if (pin == 5 || pin == 6) {
    // Sets PWM to non-inverting mode, phase correct.
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
    // Sets timer prescaler to 1024.
    TCCR0B = (1 << CS02);
  }

  // If pin is 9 or 10, use timer 1.
  else if (pin == 9 || pin == 10) {
    // Sets PWM to non-inverting mode, phase corect.
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    // Sets timer prescaler to 1024.
    TCCR1B = (1 << CS12);
  }
  // If pin is 3 or 11, use timer 2.
  else if (pin == 3 || pin == 11) {
    // Sets PWM to non-inverting mode, phase correct.
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);
    // Sets timer prescaler to 1024.
    TCCR2B = (1 << CS22);
  }
}

/*
  Writes digital value on pin.
*/
void DigitalWrite(short pin, bool value) {
  // If pin is smaller than 8 -> writes to register PORTD.
  if (pin < 8) {
    if (value == HIGH) {
      // Writes logical 1 to pin of PORTD register.
      PORTD |= 1 << pin;
    }

    else {
      // Writes logical 0 to pin of PORTD register.
      PORTD &= !(1 << pin);
    }
  }

  // Else if pin is smaller than 14 -> writes to register PORTB.
  else if (pin < 14) {
    // Separates 3 LSB from pin.
    pin = pin & 0b00000111;

    if (value == HIGH) {
      // Writes logical 1 to pin of PORTB register.
      PORTB |= 1 << pin;
    }

    else {
      // Writes logical 0 to pin of PORTB register.
      PORTB &= !(1 << pin);
    }
  }

  // Else if pin is smaller than 22 -> writes to register PORTC.
  else if (pin < 22) {
    // Separates 3 LSB from pin.
    pin = (pin - 2) & 0b00000111;

    if (value == HIGH) {
      PORTC |= 1 << pin;
    }

    else {
      PORTC &= !(1 << pin);
    }
  }
}

/*
  Reads digital value of pin.
*/
bool DigitalRead(short pin) {
  // If pin is smaller than 8 -> reads register PIND.
  if (pin < 8) {
    // Reads from PIND register.
    return (PIND & (1 << pin)) == 1 << pin;
  }

  // Else if pin is smaller than 14 -> reads register PINB.
  else if (pin < 14) {
    // Separates 3 LSB from pin.
    pin = pin & 0b00000111;

    // Reads from PINB register.
    return (PINB & (1 << pin)) == 1 << pin;
  }

  // Else if pin is smaller than 22 -> reads register PINC.
  else if (pin < 22) {
    // Separates 3 LSB from pin.
    pin = (pin - 2) & 0b00000111;

    // Reads from PINC register.
    return (PINC & (1 << pin)) == 1 << pin;
  }


  // If invalid pin is given, return false.
  return false;
}

/*
  Writes PWM signal to pin.
*/
void AnalogWrite(short pin, short value) {
  if (pin == 5 || pin == 6) {
    if (pin == 5) {
      // Writes duty cycle to channel B of timer 0.
      OCR0B = value;
    }
    else {
      // Writes duty cycle to channel A of timer 0.
      OCR0A = value;
    }
  }

  // If pin is 9 or 10, use timer 1.
  else if (pin == 9 || pin == 10) {
    if (pin == 9) {
      // Writes duty cycle to channel A of timer 1.
      OCR1A = value;
    }
    else {
      // Writes duty cycle to channel B of timer 1.
      OCR1B = value;
    }
  }
  // If pin is 3 or 11, use timer 2.
  else if (pin == 3 || pin == 11) {
    if (pin == 3) {
      // Writes duty cycle to channel B of timer 2.
      OCR2B = value;
    }
    else {
      // Writes duty cycle to channel A of timer 2.
      OCR2A = value;
    }
  }
}

/*
  Reads analog value from analog pin.
*/
int AnalogRead(short channel) {
  // Sets AREF voltage to 5V.
  ADMUX |=  (1 << REFS0);
  // Enables ADC and sets prescaler to 128.
  ADCSRA |= (1 << ADEN) | (1 <<ADPS2) | (1 <<ADPS1)  | (1 <<ADPS0);

  // Sets ADC channel.
  ADMUX |= channel & 0b1111;

  // Starts conversion.
  ADCSRA |= (1 << ADSC);

  // Waits until conversion finishes.
  while(ADCSRA & (1 << ADSC));

  // Returns measured value.
  return ADC;
}
