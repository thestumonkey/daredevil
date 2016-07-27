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

#ifndef include_defs_h
#define include_defs_h

// OLED
#define OLED_DC			8
#define OLED_CS			5
#define OLED_CLK		13
#define OLED_MOSI		11
#define OLED_RESET		10

// Ultrasonic
#define ECHOPIN                 11 // Echo Pin
#define TRIGPIN                 12 // Trigger Pin

// Pin values
#define LRA_SEL		6
#define ERM_SEL		A3
#define GRIP_SEL	3
#define SRC_SEL		4

#define DRV_2605_EN		7
#define PWM_OUT			9

// Motors
#define NUM_MOTORS		4

#define KEY_MOTOR_LRA	        3
#define KEY_MOTOR_ERM	        0

#define GR_SEL1		        A0
#define GR_SEL2		        A1


// Input
#define OVCF			0x8000
#define IRQ_PIN			2
// Electrodes
#define ELE0			0x0001    
#define ELE1			0x0002    
#define ELE2			0x0004    
#define ELE3			0x0008    
#define ELE4			0x0010    
#define ELE5			0x0020    
#define ELE6			0x0040    
#define ELE7			0x0080
#define ELE8			0x0100
#define ELE9			0x0200
#define ELE10			0x0400
#define ELE11			0x0800

// LEFT = (ELE8 ELE7 ELE6)(256 128 64)
// SELECT = (ELE5 ELE4 ELE3)(32 16 8)
// RIGHT = (ELE2 ELE1 ELE0)(4 2 1)

// BACK = (ELE9)(512)
// PLAY = (ELE10)(1024)
// PMD = (ELE11)(2048)

// Keys
#define LEFT			ELE7    // this is not physical key code it is only a number for key recognition 
#define SELECT		        ELE4
#define RIGHT			ELE1

#define BACK			ELE9
#define PLAY			ELE10
#define PMD_BUTTON		ELE11

#define LEFT_KEY			(ELE8 | ELE7 | ELE6)
#define SELECT_KEY		        (ELE5 | ELE4 | ELE3)
#define RIGHT_KEY			(ELE2 | ELE1 | ELE0)

//#define LEFT			ELE7
//#define SELECT			ELE4
//#define RIGHT			ELE1


// DRV2605
#define DRV2605_ADDR_WR	0xB4
#define DRV2605_ADDR_RD	0xB5

// Visuals
#define ANIM_SPEED		4
#define LINE_BUF_LEN	22

// Key infos
#define NUM_KEYS		6
//#define NUM_KEYS		12

#define LONG_PRESS		500	// Time for a long press in ms
#define LONG_PRESS_EFF	16
#define SHORT_PRESS_EFF	25
#define CALED_LIB		4
#define CALED_EFFECT	5

// EEPROM saved settings
#define MOTOR_VALS_ADDR	0x00	// Address of the calibration bitmask
#define MOTOR_AC_ADDR	0x01	// Address to start saving autocal vals
#define MOTOR_AC_SIZE	0x03	// 3 bytes per autocal
#define MOTOR_AC_LEN	0x0C	// MOTOR_AC_SIZE * NUM_MOTORS

#endif // include_defs_h
