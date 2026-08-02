#pragma once

#include <stdio.h>

/* Trace lines compared by tracecmp. "=" lines must match exactly. On "~tol" lines every number may differ
   by tol and every other token must match. Any other output, such as library messages, is ignored. */
#define EXACT(...) (printf("= "), printf(__VA_ARGS__), putchar('\n'))
#define APPROX(tol, ...) (printf("~%g ", (double)(tol)), printf(__VA_ARGS__), putchar('\n'))
