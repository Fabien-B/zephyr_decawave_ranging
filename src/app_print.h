#pragma once

#include <stdarg.h>

int pdec_printf(const char *fmt, ...);
int pdec_vprintf(const char *fmt, va_list args);

#define printf pdec_printf
