/*
  BinaryOperators.h - Library for various bit manipulation operations.
  Created by Vasco G. Sampaio, April 2, 2020.
  Released into the public domain.
*/

#ifndef Binaryoperators_h
#define Binaryoperators_h

void int2bytearr(byte * arr, int integer, int len) {
  for (int i = 0; i < len; i++)
    arr[i] = 0xFF & integer >> i * 8;
}

void bytearr2int (byte * arr, int& integer, int len) {
  for (int i = 0; i < len; i++)
    integer |= arr[i] << i * 8;
}

#endif
