#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

int arrLen(int *arr) {
  int size = *(&arr + 1) - arr;
  return size;
}
