#include <emscripten.h>
#include <stdio.h>


EMSCRIPTEN_KEEPALIVE
const char* hello_world() { return "Hello from Wasm!"; }

EMSCRIPTEN_KEEPALIVE
int add(int a, int b) { return a + b; }
