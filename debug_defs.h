// This Software is part of the Precision Microdrives Haptic Shield demo and is
// provided under the MIT License

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to 
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#ifndef include_debug_defs_h
#define include_debug_defs_h

#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

const char Debug_String_00[] PROGMEM = "Mem Dump:";
const char Debug_String_01[] PROGMEM = " 0x";
const char Debug_String_02[] PROGMEM = "Precision Microdrives";
const char Debug_String_03[] PROGMEM = "Firmware v1";

const char * const DBG_STRS[] PROGMEM = {
	&Debug_String_00[ 0 ],
	&Debug_String_01[ 0 ],
	&Debug_String_02[ 0 ],
	&Debug_String_03[ 0 ],
};

#endif // include_debug_defs_h
