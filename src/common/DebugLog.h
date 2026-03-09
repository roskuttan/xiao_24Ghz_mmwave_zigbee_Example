#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

#ifndef DEBUG_LOGS
#define DEBUG_LOGS 1
#endif

#if DEBUG_LOGS
static inline void DBG_BEGIN(unsigned long b) {
  Serial.begin(b);
}

static inline void DBG(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  char buf[256];
  vsnprintf(buf, sizeof(buf), fmt, ap);
  Serial.print(buf);
  va_end(ap);
}

#define DBGLN(...) do { DBG(__VA_ARGS__); Serial.println(); } while (0)
#else
#define DBG_BEGIN(...)  ((void)0)
#define DBG(...)        ((void)0)
#define DBGLN(...)      ((void)0)
#endif

