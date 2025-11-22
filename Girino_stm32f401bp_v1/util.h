/*
 *
 * https://github.com/greiman/SdFat/blob/master/src/FreeStack.h
 * Copyright (c) 2011-2018 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *  Provided as-is, No warranties of any kind, Use at your own risk
 *
 */

#ifndef UTIL_H_
#define UTIL_H_

extern "C" char* sbrk(int incr);
static int FreeStack() {
  char top = 't';
  return &top - reinterpret_cast<char*>(sbrk(0));
}

#endif
